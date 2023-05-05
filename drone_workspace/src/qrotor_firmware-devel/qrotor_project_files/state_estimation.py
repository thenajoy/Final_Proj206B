#!/usr/bin/env python
import numpy as np
from scipy.spatial import cKDTree


class QuadObserver():
    def __init__(self):
        """
        Init function for state observer for a planar quadrotor

        Args:
            dynamics (Dynamics): Dynamics object instance
            mean (float, optional): Mean for gaussian noise. Defaults to None.
            sd (float, optional): standard deviation for gaussian noise. Defaults to None.
        """
        self.pos = np.zeros((3, 1))
        self.vel = np.zeros((3, 1))
        self.accel = np.zeros((3, 1))
        self.AR_pos = np.zeros((3, 1))
    
    def get_pos(self):
        """
        Returns a potentially noisy measurement of JUST the position of the Qrotor mass center
        Returns:
            3x1 numpy array, observed position vector of system
        """
        # print(f'current pos: {self.pos}')
        return self.pos
    
    def get_vel(self):
        """
        Returns a potentially noisy measurement of JUST the spatial velocity of the Qrotor mass center
        Returns:
            3x1 numpy array, observed velocity vector of system
        """
        # print(f'current vel: {self.vel}')
        return self.vel

    def get_accel(self):
        """
        Returns a potentially noisy measurement of JUST the spatial acceleration of the Qrotor mass center
        Returns:
            3x1 numpy array, observed acceleration vector of system
        """
        return self.accel

    def set_pos_vel(self, quad_state0, quad_state1):
        self.pos, self.vel = quad_state0, quad_state1

    def set_accel(self, quad_acc):
        self.accel = quad_acc

    def set_AR_pos(self, x, y, z):
        self.AR_pos = np.array([x, y, z]).reshape((3, 1))

    def get_AR_pos(self):
        return self.AR_pos