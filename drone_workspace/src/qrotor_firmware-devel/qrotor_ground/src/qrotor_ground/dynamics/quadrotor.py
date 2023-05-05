import imp
import re
import time
import math
import IPython
from casadi.casadi import project
import numpy as np
from numpy.core.defchararray import array
import scipy as sp
from quadprog import solve_qp
from scipy.spatial.transform import Rotation as rotation
from scipy.linalg import block_diag as blkdiag

from qrotor_ground.utils.flat2state import Flat2State
from qrotor_ground.utils.trajectories.basics import setpoint, circleXY

from control import lqr
import casadi
from enum import Enum

from rosgraph.names import SEP
import rospy


class EVENT(Enum):
    REQUEST_IDLE = -1
    REQUEST_TAKEOFF = 0
    REQUEST_LAND = 1
    REQUEST_TRAJECTORY = 2
    REQUEST_SETPOINT = 3


class MODE(Enum):
    IDLE = 0
    TAKEOFF = 1
    LAND = 2
    SETPOINT = 3
    TRAJECTORY = 4

def flat_variable():
    traj = dict()
    traj['x'] = np.zeros(3)
    traj['dx'] = np.zeros(3)
    traj['d2x'] = np.zeros(3)
    traj['d3x'] = np.zeros(3)
    traj['d4x'] = np.zeros(3)
    traj['d5x'] = np.zeros(3)
    traj['d6x'] = np.zeros(3)
    return traj

class Quadrotor(object):
    """
    Quadrotor dynamics on SE3
    """
    def __init__(self, h=0.1, mass=0.775, CTRL='pd'):
        self._e1 = np.array([1.0, 0.0, 0.0])
        self._e2 = np.array([0.0, 1.0, 0.0])
        self._e3 = np.array([0.0, 0.0, 1.0])
        self._g = 9.81
        self._mass = mass
        self._inertia = np.array([0.0023, 0.0023, 0.004])
        self._inertia_matrix = np.array(
            [[0.0023, 0., 0.], [0., 0.0023, 0.], [0., 0., 0.004]])
        eig, _ = np.linalg.eig(self._inertia_matrix)
        self.J = self._inertia_matrix
        self.J_scale = (1 / np.min(eig)) * self._inertia_matrix
        self.J_inv = np.linalg.inv(self._inertia_matrix)

        self._max_force = 100
        self._max_torque = 10
        self._max_thrust = np.array([20.0, 20.0, 40.0])
        self._min_thrust = np.array([-20, -20, 0])

        self._time_step = h

        # - - - - - - - - - -
        self._quat = [0, 0, 0, 1]
        self._rotation = rotation.from_quat(self._quat)
        self._state = {'position': np.zeros((3)),
                       'velocity': np.zeros((3)),
                       'orientation': np.eye(3),
                       'ang_vel': np.zeros((3))}
        self._gains = {'position': np.array([4., 4., 8.]),
                       'velocity': np.array([3., 3, 6.]),
                       'pos-int': np.array([0.01, 0.01, 0.01]),
                       'orientation': 5 * 12. * np.array([1.0, 1.0, 1.0]),
                       'ang_vel': 5 * 2 * np.array([1.0, 1.0, 1.0])}
        self._setpoint = np.array([0., 0., 1.0])

        # - - - - - - - - - -
        self.new_setpoint = False
        self._pos_int_err = np.zeros(3)
        self._max_pos_int_err = 5*np.array([1.0, 1.0, 1.0])
        self._min_pos_int_err = -5*np.array([1.0, 1.0, 1.0])
        self._yaw_sp = 0
        self._takeoff_starttime = 0
        self._takeoff_endtime = 0
        self._takeoff_init = np.zeros(3)
        self._takeoff_final = np.zeros(3)

        self._land_starttime = 0
        self._land_endtime = 0
        self._land_init = np.zeros(3)
        self._land_final = np.zeros(3)

        # - - - - - - - - - -
        self.controller = None
        self.CTRL = CTRL
        self.set_controller(CTRL)

        self.eta1, self.epsilon1, self.c1 = 2.5, 2, 6
        self.eta2, self.epsilon2, self.c2 = 150, 4, 10

        # - - - - - - - - - -
        self.get_flat_traj = lambda t: self.set_setpoint_traj(t)
        # self.get_flat_traj = lambda t: circleXY(t, r=2, w=0.5*math.pi, c=np.array([0,0,1.]))
        self.des_state = None
        self._mode = MODE.IDLE
        self._flat_traj = flat_variable()
        self._des_state = Flat2State.quadrotor(self._flat_traj, mQ=self._mass, J=self._inertia_matrix)
        self._send = False

        self._feedfoward_force = np.zeros(3,)

        print(self._des_state)
        # - - - - - - - - - -

    def set_setpoint_traj(self, t):
        self._flat_traj = flat_variable()
        self._flat_traj['x'] = self._setpoint
        self._des_state = Flat2State.quadrotor(self._flat_traj, mQ=self._mass, J=self._inertia_matrix)

    def yaw_sp(self, t=0):
        return self._yaw_sp

    def takeoff_traj(self, t):
        # print(self._takeoff_starttime, t, self._takeoff_endtime)
        if (t > self._takeoff_endtime):
            print("takeoff time complete!")
            self._setpoint = self._takeoff_final
            self.set_setpoint_traj(t)
            
            print("requesting setpoint")
            self.set_event(t, EVENT.REQUEST_SETPOINT)
            print(self._mode)

        elif (self._takeoff_starttime <= t <= self._takeoff_endtime):
            tmp = (np.array([0., 0., 0.75]))*(t-self._takeoff_starttime)/2.
            tmp += self._takeoff_init
            self._flat_traj = flat_variable()
            self._flat_traj['x'] = tmp
            self._flat_traj['dx'] = np.array([0., 0., 0.75])/2.
            
        else:
            self._flat_traj = flat_variable()
        self._des_state = Flat2State.quadrotor(self._flat_traj, mQ=self._mass, J=self._inertia_matrix)    

    def land_traj(self, t):
        # print(self._takeoff_starttime, t, self._takeoff_endtime)
        if (t > self._land_endtime):
            self._setpoint = self._land_final
            self.set_setpoint_traj(t)
            self.set_event(t, EVENT.REQUEST_IDLE)

        elif (self._land_starttime <= t <= self._land_endtime):
            # tmp = (self._land_final-self._land_init)*(t-self._land_starttime)/10.
            # tmp += self._land_init
            # self._flat_traj = flat_variable()
            # self._flat_traj['x'] = tmp
            # self._flat_traj['dx'] = np.zeros(3)
            self._setpoint = self._land_final
            self.set_setpoint_traj(t)
            
        else:
            self._flat_traj = flat_variable()
        self._des_state = Flat2State.quadrotor(self._flat_traj, mQ=self._mass, J=self._inertia_matrix)    


    def get_desired_traj(self, t):

        if (self._mode == MODE.IDLE):
            return
        elif (self._mode == MODE.TAKEOFF):
            self.takeoff_traj(t)
            self._des_state = Flat2State.quadrotor(self._flat_traj, mQ=self._mass, J=self._inertia_matrix)

        elif (self._mode == MODE.LAND):
            self.land_traj(t)
            self._des_state = Flat2State.quadrotor(self._flat_traj, mQ=self._mass, J=self._inertia_matrix)

        elif (self._mode == MODE.SETPOINT):
            return

        elif (self._mode == MODE.TRAJECTORY):
            self._des_state = None 

        else:
            self._des_state = flat_variable()

    def request_takeoff(self,t):
        rospy.loginfo("Requesting takeoff")
        self._takeoff_init = self._state['position']
        self._takeoff_final = self._takeoff_init + np.array([0., 0., 0.75])
        self._takeoff_starttime = t
        self._takeoff_endtime = t + 2.0
        self._send = True
        self._mode = MODE.TAKEOFF
        print(self._takeoff_init, self._takeoff_final, self._takeoff_starttime, self._mode      )

    def request_land(self,t):
        print("Requesting landing")
        self._mode = MODE.LAND
        self._land_init = self._state['position']
        self._land_final = self._land_init
        self._land_final[2] = self._takeoff_init[2]
        print("landing at", self._land_final)
        self._land_starttime = t
        self._land_endtime = t +10.

    def set_event(self, t, _request):
        if (self._mode == MODE.IDLE):
            if (_request == EVENT.REQUEST_TAKEOFF):
                self.request_takeoff(t)
            else:
                print("set_event::MODE.IDLE -> CAN ONLY REQUEST_TAKEOFF")

        elif (self._mode == MODE.TAKEOFF):
            if (_request == EVENT.REQUEST_SETPOINT):
                self._mode = MODE.SETPOINT
            else:
                self._send = False
                print("set_event::MODE.IDLE -> CAN ONLY REQUEST_SETPOINT")

        elif (self._mode == MODE.LAND):
            if (_request == EVENT.REQUEST_IDLE):
                self._send = False
                self._mode = MODE.IDLE
            else:
                self._send = False
                print("set_event::MODE.LAND -> CAN ONLY REQUEST_IDLE")

        elif (self._mode == MODE.SETPOINT):
            if (_request == EVENT.REQUEST_LAND):
                self.request_land(t)

            elif (_request == EVENT.REQUEST_TRAJECTORY):
                pass
            
            else:                
                print("set_event::MODE.SETPOINT -> CAN ONLY REQUEST LAND/TRAJECTORY")
            pass

        elif (self._mode == MODE.TRAJECTORY):
            pass

        else:
            print("set_event -> UNKOWN MODE!")

    def compute_control(self, t):
        self.get_desired_traj(t)
        # print(self._des_state)
        return self.controller(self._des_state)

    def set_controller(self, CTRL='pd'):
        self.CTRL = CTRL
        if CTRL == 'pd':
            self.controller = lambda t: self.geometric_pd(t)
        elif CTRL == 'pid':
            self.controller = lambda t: self.position_pid(t)
        elif CTRL == 'clf':
            self.controller = lambda t: self.seq_clf_qp(t)
        elif CTRL == 'vbl-lqr':
            self.controller = lambda t: self.vbl_lqr(t)
        elif CTRL == 'vmpc':
            self.controller = lambda t: self.discrete_mpc(t)
            self._mpc_horizon = int(20)
            self.solver_initialized = False
            self.mpc_solver = None
            self.initial_guess = None
        elif CTRL == 'mpc':
            self.controller = lambda t: self.position_mpc(t)
            self._mpc_horizon = int(20)
            self.solver_initialized = False
            self.mpc_solver = None
            self.initial_guess = None
        else:
            self.controller = lambda t: self.geometric_pd()

    @property
    def mass(self):
        return self._mass

    @property
    def inertia(self):
        return self._inertia

    @property
    def inertia_matrix(self):
        return self._inertia_matrix

    @property
    def state(self):
        return self._state

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value):
        if self.CTRL == 'pid':
            self._pos_int_err = np.zeros(3)
        self._setpoint = value
        self.set_setpoint_traj(0)

    @property
    def quaternion(self):
        return self._rotation.as_quat()

    @quaternion.setter
    def quaternion(self, value):
        self._rotation = rotation.from_quat(self._quat)
        self._state['orientation'] = self._rotation.as_matrix()

    @property
    def position(self):
        return self._state['position']

    @position.setter
    def position(self, value):
        self._state['position'] = np.array(value)

    @property
    def velocity(self):
        return self._state['velocity']

    @velocity.setter
    def velocity(self, value):
        self._state['velocity'] = np.array(value)

    @property
    def orientation(self):
        return self._state['orientation']

    @orientation.setter
    def orientation(self, value):
        self._state['orientation'] = value
        # r = rotation.from_quat(value)
        # self._state['orientation'] = r.as_dcm()

    @property
    def ang_vel(self):
        return self._state['ang_vel']

    @ang_vel.setter
    def ang_vel(self, value):
        self._state['ang_vel'] = np.array(value)

    @property
    def des_position(self):
        return self._des_state['position']

    @des_position.setter
    def des_position(self, position):
        self._des_state['position'] = position

    @property
    def send(self):
        return self._send

    @property
    def feedforward_force(self):
        return self._feedfoward_force

    @feedforward_force.setter
    def feedforward_force(self, value):
        self._feedfoward_force = value



    def hat(self, vector):
        return np.array([[0., -vector[2], vector[1]],
                         [vector[2], 0., -vector[0]],
                         [-vector[1], vector[0], 0.]])

    def vee(self, matrix):
        return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])

    def geometric_pd(self, des_state):
        """
        geometric control for quadrotor on SE3
        """

        # position control
        ex = self._state['position'] - des_state['xQ']
        ev = self._state['velocity'] - des_state['vQ']
        Fpd = -self._gains['position'] * ex - self._gains['velocity'] * ev
        Fff = self._mass * (self._g * self._e3 + des_state['aQ'])
        thrust_force = Fpd + Fff
        norm_thrust = np.linalg.norm(thrust_force)

        # computing command attitude
        b1d = self._e1
        b3 = thrust_force / norm_thrust
        b3_b1d = np.cross(b3, b1d)
        norm_b3_b1d = np.linalg.norm(b3_b1d)
        b1 = (-1 / norm_b3_b1d) * np.cross(b3, b3_b1d)
        b2 = np.cross(b3, b1)
        Rd = np.hstack([np.expand_dims(b1, axis=1), np.expand_dims(
            b2, axis=1), np.expand_dims(b3, axis=1)])

        R = self._state['orientation']
        Omega = self._state['ang_vel']
        Omegad = des_state['Omega']
        dOmegad = des_state['dOmega']

        # attitude control
        tmp = 0.5 * (np.matmul(Rd.T, R) - np.matmul(R.T, Rd))
        eR = np.array([tmp[2, 1], tmp[0, 2], tmp[1, 0]])  # vee-map
        eOmega = Omega - np.matmul(np.matmul(R.T, Rd), Omegad)

        M = -self._gains['orientation'] * eR - self._gains['ang_vel'] * \
            eOmega + np.cross(Omega, self._inertia * Omega)
        M += -1 * self._inertia_matrix.dot(
            np.matmul(self.hat(Omega), np.matmul(R.T, np.matmul(Rd, Omegad))) - np.matmul(R.T, np.matmul(Rd,
                                                                                                         dOmegad)))  # ignoring this for since Omegad is zero
        f = thrust_force.dot(R[:, 2])

        # f, M = des_state['f'], des_state['M']

        # compute lyapunov function
        eta1, epsilon1, c1 = 2.5, 2, 6
        V1 = 0.5 * ev.dot(ev) + epsilon1 * ex.dot(ev) + 0.5 * c1 * ex.dot(ex)

        eta2, epsilon2, c2 = 150, 4, 20
        V2 = 0.5 * eOmega.dot(np.matmul(self.J_scale, eOmega)) + \
            epsilon2 * eR.dot(eOmega) + 0.5 * c2 * eR.dot(eR)

        return thrust_force, (f, M), (V1, V2)

    def position_pid(self, des_state):
        """
        geometric control for quadrotor on SE3
        """
        # position control
        ex = self._state['position'] - des_state['xQ']
        ev = self._state['velocity'] - des_state['vQ']
        Fpd = -self._gains['position'] * ex - self._gains['velocity'] * \
            ev - self._gains['pos-int']*self._pos_int_err
        Fff = self._mass * (self._g * self._e3 + des_state['aQ']) 
        thrust_force = Fpd + Fff + self._feedfoward_force

        # perform integral
        self._pos_int_err += self._time_step*ex
        # add bounds to the thrust force
        # TODO add antiwind
        self._pos_int_err = np.maximum(np.minimum(
            self._pos_int_err, self._max_pos_int_err), self._min_pos_int_err)
        thrust_force = np.maximum(np.minimum(
            thrust_force, self._max_thrust), self._min_thrust)
        return thrust_force, (0, np.zeros(0)), (0, 0)

    def create_position_mpc_solver(self):
        x = casadi.MX.sym('x', 6)
        u = casadi.MX.sym('u', 3)

        N = self._mpc_horizon
        h = self._time_step
        Q_ = np.diag([20., 20., 10., 20., 20., 20.])
        R_ = 1 * np.eye(3)

        O, I = np.zeros((3, 3)), np.eye(3)

        ar1 = np.concatenate((I, I*h), axis=1)
        ar2 = np.concatenate((O, I), axis=1)
        A = np.concatenate((ar1, ar2), axis=0)
        B = np.concatenate((I*(0.5*h*h), I*h), axis=0)

        Ak = casadi.MX(A)
        Bk = casadi.MX(B)
        rhs = Ak @ x + Bk @ (u*casadi.MX(1/self.mass) -
                             casadi.MX(np.array([0, 0, 9.81])))
        fdyn = casadi.Function('fdyn', [x, u], [rhs])
        U = casadi.MX.sym('U', 3, N)
        X = casadi.MX.sym('X', 6, N + 1)
        x0 = casadi.MX.sym('x0', 6)
        xd = casadi.MX.sym('xd', 6)
        ud = casadi.MX.sym('ud', 3)

        st = X[:, 0]
        g = []
        # initial state
        g.append(st - x0)
        objective = 0

        Q = casadi.MX(Q_)
        R = casadi.MX(R_)

        Pmat = sp.linalg.solve_discrete_are(A, B, Q_, R_)

        for k in range(N):
            # decision variables
            st = X[:, k]
            con = U[:, k]
            st_next = X[:, k + 1]
            # dynamics (xk+1 = Ak xk + Bk uk)
            f_val = fdyn(st, con)
            g.append(st_next - f_val)
            # cost function: quadratic cost
            objective = objective + \
                (st-xd).T @ Q @ (st-xd) + (con-ud).T @ R @ (con-ud)
        # terminal-cost
        objective += (X[:, -1]-xd).T @ casadi.MX(Pmat) @ (X[:, -1]-xd)
        opt_vars = casadi.vertcat(casadi.MX.reshape(
            X, (6 * (N + 1), 1)), casadi.MX.reshape(U, (3 * N, 1)))
        opt_param = casadi.vertcat(x0, xd, ud)

        nlp = {}  # NLP declaration
        nlp['x'] = opt_vars  # decision vars
        nlp['f'] = objective  # objective
        nlp['g'] = casadi.vertcat(*g)
        nlp['p'] = opt_param

        opts = {}
        opts['print_time'] = 0
        opts['ipopt'] = {}
        opts['ipopt']['acceptable_tol'] = 1e-3
        opts['ipopt']['acceptable_obj_change_tol'] = 1e-3
        opts['ipopt']['print_level'] = 0
        return casadi.nlpsol('mpc_solver', 'ipopt', nlp, opts)

    def position_mpc(self, des_state):
        N = self._mpc_horizon
        N_OPT_VARS = 6 * (N + 1) + 3 * N
        N_CONST = 6*(N+1)

        if not self.solver_initialized:
            self.mpc_solver = self.create_position_mpc_solver()
            self.solver_initialized = True
            self.initial_guess = np.zeros((N_OPT_VARS,))

        # compute errors
        pd = des_state['xQ']
        vd = des_state['vQ']
        # ud = self.mass*(des_state['aQ']+np.array([0.0, 0.0, 9.81]))
        ud = self._mass * (self._g * self._e3 + des_state['aQ'])
        p = self._state['position']
        v = self._state['velocity']

        params = np.concatenate((p, v, pd, vd, ud), axis=0)

        res = self.mpc_solver(x0=self.initial_guess, ubg=np.zeros(
            (N_CONST,)), lbg=np.zeros((N_CONST,)), p=params)
        sol = np.array(res['x'])
        self.initial_guess = sol
        u_mpc = sol[6 * (N + 1):6 * (N + 1) + 3].ravel()
        return u_mpc, (0, np.zeros(3)), (0, 0)

    def seq_clf_qp(self, des_state):
        """
        Sequential CLF-MinNorm Controller without CBF
        """
        ##################################################
        #  first level of QP : compute the virtual force #
        ##################################################
        # eta1, epsilon1, c1 = 2.5, 2, 6
        thrust_force, V1 = self.pos_clf_qp()

        ##################################################
        # second level of QP : orientation adjustment    #
        # without considering the external obstacle      #
        ##################################################
        # computing command attitude
        norm_thrust = np.linalg.norm(thrust_force)
        b1d = self._e1
        b3 = thrust_force / norm_thrust
        b3_b1d = np.cross(b3, b1d)
        norm_b3_b1d = np.linalg.norm(b3_b1d)
        b1 = (-1 / norm_b3_b1d) * np.cross(b3, b3_b1d)
        b2 = np.cross(b3, b1)
        Rd = np.hstack([np.expand_dims(b1, axis=1), np.expand_dims(
            b2, axis=1), np.expand_dims(b3, axis=1)])

        R = self._state['orientation']
        Omega = self._state['ang_vel']
        Omegad = self._des_state['Omega']
        dOmegad = self._des_state['dOmega']

        # attitude control
        tmp = 0.5 * (np.matmul(Rd.T, R) - np.matmul(R.T, Rd))
        eR = np.array([tmp[2, 1], tmp[0, 2], tmp[1, 0]])  # vee-map
        eOmega = Omega - np.matmul(np.matmul(R.T, Rd), Omegad)

        M = -self._gains['orientation'] * eR - self._gains['ang_vel'] * \
            eOmega + np.cross(Omega, self._inertia * Omega)
        M += -1 * self._inertia_matrix.dot(
            np.matmul(self.hat(Omega), np.matmul(R.T, np.matmul(Rd, Omegad))) - np.matmul(R.T, np.matmul(Rd,
                                                                                                         dOmegad)))  # ignoring this for since Omegad is zero
        f = thrust_force.dot(R[:, 2])

        V2 = 0.5 * eOmega.dot(np.matmul(self.J_scale, eOmega)) + self.epsilon2 * eR.dot(
            eOmega) + 0.5 * self.c2 * eR.dot(eR)

        return (f, M), (V1, V2)

    def pos_clf_qp(self):

        # position and velocity error
        ex = self._state['position'] - self.des_state['xQ']
        ev = self._state['velocity'] - self.des_state['vQ']
        des_acc = self.des_state['aQ']

        return self.compute_pos_clf_qp(ex, ev, des_acc)

    def compute_pos_clf_qp(self, ex, ev, des_acc):
        """
        Position CLF qp
        computes the desired thrust 
        """
        V1 = 0.5 * ev.dot(ev) + self.epsilon1 * ex.dot(ev) + \
            0.5 * self.c1 * ex.dot(ex)
        LgV1 = ev + self.epsilon1 * ex
        LfV1 = self.epsilon1 * \
            ev.dot(ev) + self.c1 * ev.dot(ex) - LgV1.dot(des_acc)

        A1 = np.array([LgV1])
        b1 = np.array([-LfV1 - self.eta1 * V1])
        H1 = np.diag([5, 5, 1])
        f1 = -1 * self.des_state['aQ']  # np.zeros((3,))

        acc_opt, _, _, _, _, _ = solve_qp(H1.astype(np.double), f1.astype(np.double), -1 * A1.T.astype(np.double),
                                          -1 * b1.astype(np.double))
        fd = self._mass * (acc_opt + self._g * self._e3)

        return fd, V1

    def att_clf_qp(self, fd):
        Rd = self.des_state['R']
        Omegad = self.des_state['Omega']
        dOmegad = self.des_state['dOmega']

        # computing command attitude
        b1d = Rd[:, 0]
        b3c = fd / np.linalg.norm(fd)
        b3c_b1d = np.cross(b3c, b1d)
        norm_b3c_b1d = np.linalg.norm(b3c_b1d)
        b1c = (-1 / norm_b3c_b1d) * np.cross(b3c, b3c_b1d)
        b2c = np.cross(b3c, b1c)
        Rc = np.hstack([np.expand_dims(b1c, axis=1), np.expand_dims(
            b2c, axis=1), np.expand_dims(b3c, axis=1)])

        R = self.state['orientation']
        Omega = self.state['ang_vel']

        return self.compute_att_clf_qp(R, Rc, Omega, Omegad, dOmegad)

    def compute_att_clf_qp(self, R, Rc, Omega, Omegad, dOmegad):
        """
        Attitude clf qp
        """
        Omega_hat = self.hat(Omega)

        # compute the orientation error
        tmp = 0.5 * (np.matmul(Rc.T, R) - np.matmul(R.T, Rc))
        eR = np.array([tmp[2, 1], tmp[0, 2], tmp[1, 0]])  # vee-map
        eOmega = Omega - np.matmul(np.matmul(R.T, Rc), Omegad)
        dR = np.matmul(R, Omega_hat)
        dRc = np.matmul(Rc, self.hat(Omegad))
        deR = 0.5 * (self.vee(np.matmul(dRc.T, R) - np.matmul(R.T, dRc)) + self.vee(
            np.matmul(Rc.T, dR) - np.matmul(dR.T, Rc)))

        V2 = 0.5 * eOmega.dot(np.matmul(self.J_scale, eOmega)) + self.epsilon2 * eR.dot(
            eOmega) + 0.5 * self.c2 * eR.dot(eR)
        LgV2 = np.matmul(eOmega.T, self.J_scale) + self.epsilon2 * eR.T
        LfV2 = (self.epsilon2 * eOmega + self.c2 * eR).dot(deR) - LgV2.dot(
            np.matmul(dR.T, np.matmul(Rc, Omegad)) + np.matmul(R.T, np.matmul(Rc, dOmegad)))

        H2 = np.diag([1, 1, 1, 4e2])
        f2 = np.zeros(4)
        f2 = -1 * np.append(self.des_state['dOmega'], np.array([0]))
        A2 = np.array([np.append(LgV2, -1)])
        b2 = np.array([-LfV2 - self.eta2 * V2])

        dOmega_opt, _, _, _, _, _ = solve_qp(H2.astype(np.double), f2.astype(np.double), -1 * A2.T.astype(np.double),
                                             -1 * b2.astype(np.double))
        dOmega = dOmega_opt[:3]

        M = self._inertia_matrix.dot(dOmega) + np.matmul(Omega_hat,
                                                         np.matmul(self._inertia_matrix, self._state['ang_vel']))
        return M, V2

    def get_var_linear_dynamics(self, ref):
        A = []
        B = []
        O, I = np.zeros((3, 3)), np.eye(3)

        a23 = -1 * (ref['f'] / self._mass) * \
            np.matmul(ref['R'], self.hat(self._e3))
        a44 = np.matmul(np.linalg.inv(self._inertia_matrix),
                        self.hat(self._inertia_matrix.dot(ref['Omega'])) - np.matmul(self.hat(ref['Omega']),
                                                                                     self._inertia_matrix))

        ar1 = np.concatenate((O, I, O, O), axis=1)
        ar2 = np.concatenate((O, O, a23, O), axis=1)
        ar3 = np.concatenate((O, O, -1 * self.hat(ref['Omega']), I), axis=1)
        ar4 = np.concatenate((O, O, O, a44), axis=1)

        A = np.concatenate((ar1, ar2, ar3, ar4), axis=0)

        bc1 = np.concatenate((np.zeros((3, 1)), (1 / self._mass) * np.matmul(ref['R'], np.array([self._e3]).T),
                              np.zeros((3, 1)), np.zeros((3, 1))), axis=0)
        bc2 = np.concatenate(
            (O, O, O, np.linalg.inv(self._inertia_matrix)), axis=0)

        B = np.concatenate((bc1, bc2), axis=1)
        return A, B

    def vbl_lqr(self, t):
        des_state = self.get_desired_traj(t)

        A, B = self.get_var_linear_dynamics(des_state)
        Q = blkdiag(5 * np.eye(6), 0.75 * np.eye(6))
        R = 0.01 * np.eye(4)
        K, _, _ = lqr(A, B, Q, R)

        ex = self._state['position'] - des_state['xQ']
        ev = self._state['velocity'] - des_state['vQ']

        R = self._state['orientation']
        Omega = self._state['ang_vel']

        Rd = des_state['R']
        Omegad = des_state['Omega']

        # attitude control
        tmp = 0.5 * (np.matmul(Rd.T, R) - np.matmul(R.T, Rd))
        eR = np.array([tmp[2, 1], tmp[0, 2], tmp[1, 0]])  # vee-map
        eOmega = Omega - np.matmul(np.matmul(R.T, Rd), Omegad)

        err = np.array([np.concatenate((ex, ev, eR, eOmega))]).T

        u_feedback = -1 * np.matmul(K, err)

        f = des_state['f'] + np.array(u_feedback[0, 0]).ravel()
        M = des_state['M'] + np.array(u_feedback[1:, 0]).ravel()

        # compute lyapunov function
        eta1, epsilon1, c1 = 2.5, 2, 6
        V1 = 0.5 * ev.dot(ev) + epsilon1 * ex.dot(ev) + 0.5 * c1 * ex.dot(ex)

        eta2, epsilon2, c2 = 150, 4, 20
        V2 = 0.5 * eOmega.dot(np.matmul(self.J_scale, eOmega)) + \
            epsilon2 * eR.dot(eOmega) + 0.5 * c2 * eR.dot(eR)

        return (f, M), (V1, V2)

    def get_var_linear_discrete_dynamics(self, h, ref):
        """
        compute variation linearized discrete dynamics for quadrotor on SE3
        """
        O3, I3 = np.zeros((3, 3)), np.eye(3)

        Omegak_des = ref['Omega']
        dOmegad = ref['dOmega']

        Fkd = sp.linalg.expm(h * self.hat(Omegak_des))
        S = self.J_inv @ (Fkd.T @
                          (h * self.hat(self.inertia_matrix @ Omegak_des)))

        Amat_1 = np.concatenate((I3, h * I3, O3, O3), axis=1)
        Amat_2 = np.concatenate((O3, I3, O3, O3), axis=1)
        Amat_3 = np.concatenate((O3, O3, Fkd.T, h * Fkd.T), axis=1)
        Amat_4 = np.concatenate((O3, O3, O3, S), axis=1)

        Amat = np.concatenate((Amat_1, Amat_2, Amat_3, Amat_4))

        Bmat_1 = np.concatenate((0.5 * h ** 2 * I3 / self.mass, O3), axis=1)
        Bmat_2 = np.concatenate((h * I3 / self.mass, O3), axis=1)
        Bmat_3 = np.concatenate((O3, O3), axis=1)
        Bmat_4 = np.concatenate((O3, h * self.J_inv), axis=1)
        Bmat = np.concatenate((Bmat_1, Bmat_2, Bmat_3, Bmat_4))

        return Amat, Bmat

    def discrete_mpc_solver(self, des_state):
        delx_sym = casadi.MX.sym('delx', 12)
        delu_sym = casadi.MX.sym('delu', 4)

        N = self._mpc_horizon
        # Ak_, Bk_ = self.get_var_linear_discrete_dynamics(0.01, des_state)
        A, B = self.get_var_linear_dynamics(des_state)
        Ak_ = np.eye(12) + A * self._time_step
        Bk_ = B * self._time_step

        Q_ = np.diag([200., 200., 1000., 200., 200., 200.,
                     200., 500., 200., 200., 200., 200.])
        R_ = 0.01 * np.eye(4)

        Ak = casadi.MX(Ak_)
        Bk = casadi.MX(Bk_)
        rhs = Ak @ delx_sym + Bk @ delu_sym
        fdyn = casadi.Function('fdyn', [delx_sym, delu_sym], [rhs])
        U = casadi.MX.sym('U', 4, N)
        X = casadi.MX.sym('X', 12, N + 1)
        opt_param = casadi.MX.sym('x0', 12)

        st = X[:, 0]
        g = []
        # initial state
        g.append(st - opt_param)
        objective = 0

        Q = casadi.MX(Q_)
        R = casadi.MX(R_)

        Pmat = sp.linalg.solve_discrete_are(Ak_, Bk_, Q_, R_)

        for k in range(N):
            # decision variables
            st = X[:, k]
            con = U[:, k]
            st_next = X[:, k + 1]
            # dynamics (xk+1 = Ak xk + Bk uk)
            f_val = fdyn(st, con)
            g.append(st_next - f_val)
            # cost function: quadratic cost
            objective = objective + st.T @ Q @ st + con.T @ R @ con
        # terminal-cost
        objective += X[:, -1].T @ casadi.MX(Pmat) @ X[:, -1]

        opt_vars = casadi.vertcat(casadi.MX.reshape(
            X, (12 * (N + 1), 1)), casadi.MX.reshape(U, (4 * N, 1)))

        nlp = {}  # NLP declaration
        nlp['x'] = opt_vars  # decision vars
        nlp['f'] = objective  # objective
        nlp['g'] = casadi.vertcat(*g)
        nlp['p'] = opt_param

        opts = {}
        opts['print_time'] = 0
        opts['ipopt'] = {}
        opts['ipopt']['acceptable_tol'] = 1e-3
        opts['ipopt']['acceptable_obj_change_tol'] = 1e-3
        opts['ipopt']['print_level'] = 0
        return casadi.nlpsol('F', 'ipopt', nlp, opts)

    def discrete_mpc(self, t):

        N = self._mpc_horizon
        N_OPT_VARS = 12 * (N + 1) + 4 * N

        des_state = self.get_desired_traj(t)
        if not self.solver_initialized:
            des_state = self.get_desired_traj(0)
            self.mpc_solver = self.discrete_mpc_solver(des_state)
            self.solver_initialized = True
            self.initial_guess = np.zeros((N_OPT_VARS,))

        # compute errors
        ex = self._state['position'] - des_state['xQ']
        ev = self._state['velocity'] - des_state['vQ']

        R = self._state['orientation']
        Omega = self._state['ang_vel']
        Rd = des_state['R']
        Omegad = des_state['Omega']

        # attitude control
        tmp = 0.5 * (np.matmul(Rd.T, R) - np.matmul(R.T, Rd))
        eR = np.array([tmp[2, 1], tmp[0, 2], tmp[1, 0]])  # vee-map
        eOmega = Omega - np.matmul(np.matmul(R.T, Rd), Omegad)

        err = np.array([np.concatenate((ex, ev, eR, eOmega))]).T

        res = self.mpc_solver(x0=self.initial_guess, ubg=0, lbg=0, p=err)
        sol = np.array(res['x'])
        self.initial_guess = sol
        u_mpc = sol[12 * (N + 1):12 * (N + 1) + 4].ravel()

        return (u_mpc[0], u_mpc[1:]), (0., 0.)
