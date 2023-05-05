#!/usr/bin/env python
import numpy as np
import time

class Trajectory:
    def __init__(self, start, end, T):
        """
        Init function for linear tracking trajectories in RN.
        Generates a smooth straight line trajectory with zero start and end velocity. Uses sinusoidal interpolation.
        Args:
            start (Nx1 numpy array): initial spatial position in N dimensions (NOT initial state vector)
            end (Nx1 numpy array): final spatial position in N dimensions
            T (float): trajectory period
        """
        self.x0 = start
        self.xF = end
        self.spatialDimn = self.x0.shape[0]
        self.published_counter = 0
        self.T = T

        self.a_0x = 0
        self.a_0y = 0
        self.a_0z = 0

        self.a_1x = 0
        self.a_1y = 0
        self.a_1z = 0

        self.a_2x = 0
        self.a_2y = 0
        self.a_2z = 0

        self.a_3x = 0
        self.a_3y = 0
        self.a_3z = 0

        self.des_pos = 0
        self.des_vel = 0

    # def pos(self, t):
    #     return self.des_pos

    # def vel(self, t):
    #     return self.des_vel
    
    def pos(self, t):
        """
        Function to get desired position at time t
        Args:
            t (float): current time
        Returns:
            (Nx1 numpy array): position coordinates for the quadrotor to track at time t
        """
        #use sinusoidal interpolation to get a smooth trajectory with zero velocity at endpoints
        if t>self.T:
            #if beyond the time of the trajectory end, return the desired position as a setpoint
            return self.xF
        des_pos = (self.xF-self.x0)/2*np.sin(t*np.pi/self.T - np.pi/2)+(self.x0+self.xF)/2
        #des_pos = a3*t**3 + a2*t**2 + a1*t + a0
        return des_pos 
    
    def vel(self, t):
        """
        Function to get the desired velocity at time t
        Inputs:
            t: current time
        Returns:
            (Nx1 Numpy array): velocity for the system to track at time t
        """
        #differentiate position
        if t>self.T:
            #If beyond the time of the trajectory end, return 0 as desired velocity
            return np.zeros((self.spatialDimn, 1))
        des_vel = (self.xF-self.x0)/2*np.cos(t*np.pi/self.T - np.pi/2)*np.pi/self.T
        #des_vel = 3*a3*t**2 + 2*a2*t + a1
        return des_vel

    # def update_pos(self, a0, a1, a2, a3, t):
    #     """
    #     Function to get desired position at time t
    #     Args:
    #         t (float): current time
    #     Returns:
    #         (Nx1 numpy array): position coordinates for the quadrotor to track at time t
    #     """
    #     #use sinusoidal interpolation to get a smooth trajectory with zero velocity at endpoints
    #     # if t>self.T:
    #     #     #if beyond the time of the trajectory end, return the desired position as a setpoint
    #     #     return self.xF
    #     des_pos = (self.xF-self.x0)/2*np.sin(t*np.pi/self.T - np.pi/2)+(self.x0+self.xF)/2
    #     #des_pos = a3*t**3 + a2*t**2 + a1*t + a0
    #     return des_pos 
    
    # def update_vel(self, a0, a1, a2, a3, t):
    #     """
    #     Function to get the desired velocity at time t
    #     Inputs:
    #         t: current time
    #     Returns:
    #         (Nx1 Numpy array): velocity for the system to track at time t
    #     """
    #     #differentiate position
    #     # if t>self.T:
    #     #     #If beyond the time of the trajectory end, return 0 as desired velocity
    #     #     return np.zeros((self.spatialDimn, 1))
    #     des_vel = (self.xF-self.x0)/2*np.cos(t*np.pi/self.T - np.pi/2)*np.pi/self.T
    #     #des_vel = 3*a3*t**2 + 2*a2*t + a1
    #     return des_vel

    def accel(self, t):
        """
        Function to get the desired acceleration at time t
        Args:
            t: current time
        Returns:
            (Nx1 Numpy array): acceleration for the system to track at time t
        """
        #differentiate acceleration
        if t>self.T:
            #If beyond the time of the trajectory end, return 0 as desired acceleration
            return np.zeros((self.spatialDimn, 1))
        des_accel = -(self.xF-self.x0)/2*np.sin(t*np.pi/self.T - np.pi/2)*(np.pi/self.T)**2
        return des_accel

    def reset(self, curr_pos, curr_vel, des_pos, t):
        """
        Updates coefficients in class parameters
        Args:
            t: current time
        """
        x_pos, y_pos, z_pos = curr_pos[0,0], curr_pos[1,0], curr_pos[2,0]
        x_vel, y_vel, z_vel = curr_vel[0,0], curr_vel[1, 0], curr_vel[2, 0]

        mat = np.array([[0, 1, 0, 0],
                        [1, 0, 0, 0],
                        [1, t, t**2, t**3],
                        [0, 1, 2*t, 3*t**2]])

        self.a_0x, self.a_1x, self.a_2x, self.a_3x = np.linalg.pinv(mat) @ np.array([[x_vel, x_pos, des_pos[0,0], 0]]).reshape((4,1))
        self.a_0y, self.a_1y, self.a_2y, self.a_3y = np.linalg.pinv(mat) @ np.array([[y_vel, y_pos, des_pos[1,0], 0]]).reshape((4,1))
        self.a_0z, self.a_1z, self.a_2z, self.a_3z = np.linalg.pinv(mat) @ np.array([[z_vel, z_pos, des_pos[2,0], 0]]).reshape((4,1))

        des_pos_x = self.update_pos(self.a_0x, self.a_1x, self.a_2x, self.a_3x, t)
        des_pos_y = self.update_pos(self.a_0y, self.a_1y, self.a_2y, self.a_3y, t)
        des_pos_z = self.update_pos(self.a_0z, self.a_1z, self.a_2z, self.a_3z, t)
        self.des_pos = np.array([[des_pos_x, des_pos_y, des_pos_z]]).reshape((3,1))

        des_vel_x = self.update_vel(self.a_0x, self.a_1x, self.a_2x, self.a_3x, t)
        des_vel_y = self.update_vel(self.a_0y, self.a_1y, self.a_2y, self.a_3y, t)
        des_vel_z = self.update_vel(self.a_0z, self.a_1z, self.a_2z, self.a_3z, t)
        self.des_vel = np.array([[des_vel_x, des_vel_y, des_vel_z]]).reshape((3,1))

        

    def get_state(self, t):
        """
        Function to get the desired position, velocity, and accel at a time t
        Inputs:
            t: current time
        Returns:
            x_d, v_d, a_d: desired position, velocity, and acceleration at time t
        """
        return self.pos(t), self.vel(t), self.accel(t)
        # return self.des_pos, self.des_vel, self.accel(t)

