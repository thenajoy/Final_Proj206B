#!/usr/bin/env python
import numpy as np
import casadi as ca
from cbf_barrier import *

"""
File containing controllers 
"""
class Controller:
    def __init__(self, observer = None, lyapunov = None, trajectory = None):
        """
        Skeleton class for feedback controllers
        Args:
            observer (Observer): state observer object
            lyapunov (LyapunovBarrier): lyapunov functions, LyapunovBarrier object
            trajectory (Trajectory): trajectory for the controller to track (could just be a constant point!)
            obstacleQueue (ObstacleQueue): ObstacleQueue object, stores all barriers for the system to avoid
            uBounds ((Dynamics.inputDimn x 2) numpy array): minimum and maximum input values to the system
        """
        #store input parameters
        self.lyapunov = lyapunov
        self.trajectory = trajectory
        self.observer = observer
        
        #store input
        self._u = None
    
    def eval_input(self, t):
        """
        Solve for and return control input
        Inputs:
            t (float): time in simulation
        Returns:
            u ((Dynamics.inputDimn x 1)): input vector, as determined by controller
        """
        self._u = np.zeros((self.observer.inputDimn, 1))
        return self._u
    
    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class paramter
        """
        return self._u
    
class PlanarQrotorPD(Controller):
    def __init__(self, observer = None, lyapunov = None, trajectory = None):
        """
        Init function for a planar quadrotor controller.

        Args:
            observer (Observer): state observer object
            lyapunov (LyapunovBarrier): lyapunov functions, LyapunovBarrier object
            trajectory (Trajectory): trajectory for the controller to track (could just be a constant point!)
            obstacleQueue (ObstacleQueue): ObstacleQueue object, stores all barriers for the system to avoid
            uBounds ((Dynamics.inputDimn x 2) numpy array): minimum and maximum input values to the system
        """
        super().__init__(observer = observer, lyapunov = lyapunov, trajectory = trajectory)
        
        #Initialize variables for the gain parameters
        self.K = 15
        #self.K=15 
        #Store quadrotor parameters from the observer
        self.m = 0.96
        # self.Ixx = self.observer.dynamics._Ixx
        self.g = 9.81 #store gravitational constant
        
        #store Euclidean basis vector
        self.e1 = np.array([[1, 0, 0]]).T
        self.e2 = np.array([[0, 1, 0]]).T
        self.e3 = np.array([[0, 0, 1]]).T
    
    def get_position_error(self, t):
        """
        Function to return the position error vector x_d - x_q
        Args:
            t (float): current time in simulation
        Returns:
            eX ((3 x 1) NumPy array): x_d - x_q based on current quadrotor state
        """
        #retrieve desired and current positions
        xD = self.trajectory.pos(t)
        # print("desired:", xD.T)
        xQ = self.observer.get_pos()
        
        #return difference
        return xD - xQ
    
    def get_velocity_error(self, t, vdCLF = None):
        """
        Function to return velocity error vector v_d - v_q
        Args:
            t (float): current time in simulation
        Returns:
            eX ((3 x 1) NumPY array): vD - vQ
        """
        if vdCLF is None:
            #retrieve desired and current velocities
            vD = self.trajectory.vel(t)
        else:
            vD = vdCLF
        vQ = self.observer.get_vel()
        # print(f'vQ: {vQ.T}')
        # vQ = np.array([0, 0, vQ[2, 0]]).reshape((3,1))
        #return difference
        return vD - vQ
    
    def eval_force_vec(self, t, vdCLF = None):
        """
        Function to evaluate the force vector input to the system using point mass dynamics.
        Args:
            t (float): current time in simulation
        Returns:
            f ((3 x 1) NumPy Array): virtual force vector to be tracked by the orientation controller
        """
        #find position and velocity error
        eX = self.get_position_error(t)
        eV = self.get_velocity_error(t, vdCLF)
        
        #calculate control input - add feedforward acceleration term
        # print(f'eV: {eV}')
        return self.m*(self.g*self.e3 + self.K*eV)

        #vel2track error is interesting, vQ should be [0,0,t]
    

class PlanarQrotorLyapunov(PlanarQrotorPD):
    """
    Class for a CLF-QP tracking controller, inherits from PlanarQrotorPD
    """
    def __init__(self, observer, lyapunov = None, trajectory = None, clfGamma = 1.5):
        """
        Init function for a planar quadrotor CLF-QP tracking controller.
        Inherits from PD class, and uses PD orientation contorl, ES-CLF-QP tracking control
        Inputs:
            clfGamma (float): constant for ES-CLF rate of convergence
        """
        super().__init__(observer, lyapunov = lyapunov, trajectory = trajectory)
        self.clfGamma = clfGamma
        self.barrierControl = BarrierControl()

    def eval_vel_vec(self, t, vD, ar_tag_pos):
        """
        Evaluates the force vector input to the system at time t
        Args:
            t (float): current time in simulatioopti.subject_to(h1_dot + (gamma1*h1) >= 0)n
        Returns:
            f ((3 x 1) NumPy Array): virtual force vector to be tracked by the orientation controller
        """
        #set up optimization problem
        opti = ca.Opti()
        
        #set up optimization variables
        v = opti.variable(3, 1) #the input here is a force vector only! Will be 3D.
        delta = opti.variable(1, 1)

        #get the CLF value and its derivative
        V = self.lyapunov.evalLyapunov(t)
        VDot = self.lyapunov.evalLyapunovDerivs(t,v,vD) #pass the input into the function

        #get the safety controls for CBF
        h1, h1_dot, h2, h2_dot, h3, h3_dot = self.barrierControl.barrier(self.observer.get_pos(), v, ar_tag_pos)

        #Apply CLF constraint to problem
        opti.subject_to(VDot <= (-self.clfGamma*V))

        #Tune CBF constants for constraints
        gamma1 = 60
        gamma2 = 40
        gamma3 = 50
        # gamma1 = 100
        # gamma2 = 100
        # gamma3 = 100

        #Tune p constant in cost function
        p = 70
        # p = 100
        #Construct H matrix and tune eigenvalues
        eig1 = 10
        eig2 = 1
        eig3 = 8 #6
        H = np.diag([eig1, eig2, eig3])

        #Apply CBF constraints
        opti.subject_to(h1_dot >= -(gamma1*h1))
        opti.subject_to(h2_dot >= -(gamma2*h2))
        opti.subject_to(h3_dot >= -(gamma3*h3))

        #Define Cost Function
        #cost = ca.mtimes(vD.T,vD)
        cost = v.T @ H @ v + p * delta**2

        #set up optimization problem
        opti.minimize(cost)
        option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
        opti.solver("ipopt", option)

        #solve optimization
        try:
            sol = opti.solve()
            uOpt = sol.value(vD) #extract optimal input
            print('uOPt', uOpt)
            solverFailed = False
        except:
            print("Solver failed!")
            solverFailed = True
            uOpt = np.zeros((3, 1)) #return a zero vector
        
        #store output in class param - add on the effect of gravity here!
        self._u = uOpt.reshape((3, 1))

        #return result
        return self._u

    def graph(self):

        #define dynamics
        def dyn(v):
        #v is velocity input, return xDot = v
            return v

        #run the simulation
        T = 5
        dT = 0.01

        #initialize position as zero vector
        x = np.zeros((3, 1))

        xHist = []
        xDesHist = []
        yHist = []
        yDesHist = []

        for i in range(int(T/dT)): 
        #calculate time
            t = dT*i

            #get desired position, velocity
            xD, vD = get_traj(t)

            #update history arrays
            xHist.append(x[0, 0])
            xDesHist.append(xD[0, 0])
            yHist.append(x[1, 0])
            yDesHist.append(xD[1, 0])
            
            #get the input to the system
            vOpti = eval_input(x, xD, vD)

            #update the position vector
            x = x + dyn(vOpti)*dT

        #plot results - x should go to 25 quadratically if all works well!
        plt.plot(xHist)
        plt.plot(xDesHist, '--')
        plt.plot(yHist)
        plt.plot(yDesHist, '--')
        plt.ylabel("State")
        plt.xlabel("Timestep")
        plt.legend(["Actual X", "Desired X", "Actual Y", "Desired Y"])
        plt.title("CLF tracking control results")
        plt.show()