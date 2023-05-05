#!/usr/bin/env python
import numpy as np

class BarrierControl:

    def __init__(self):
        self._vals = None
        self.l_outer = 1.15
        self.l_inner = 0.85
        self.radius = 0.1
        # self.observer = observer
        
    def barrier(self, pos, vel, ar_tag):
        '''
        Evaluates a barrier that is from the Face Frame in the Drone's Frame
        --------------------------------
        Input:
            u (int): inputs to the system
        Return:
            self._vals (list): list containing h, and h_Dot for 3 constraints
        '''
        qD = pos
        qD_dot = vel

        p_cone = ar_tag   #Need to know how to get the position of the AR tag for the face in the spatial frame

        #Position of the drone in the F frame
        p_drone = qD - p_cone 

        ax = np.array([[-1, 0, 0]]).reshape((3,1))
        ax_dot = np.array([[0, 0, 0]]).reshape((3,1))

        alpha_c = np.arctan(self.radius/self.l_outer)
        #Angle from axis of cone to drone
        alpha_d = np.arccos((p_drone.T @ ax)/(np.linalg.norm(p_drone)*np.linalg.norm(ax)))

        h1 = alpha_c - alpha_d
        h1_dot = 1/(np.linalg.norm(p_drone)*np.linalg.norm(ax))*((qD_dot.T @ ax + ax_dot.T @ qD)/(np.sqrt(1 - ((qD.T @ ax)/(np.linalg.norm(qD)*np.linalg.norm(ax))**2))))

        h2 = self.l_outer - (qD.T @ ax)/np.linalg.norm(ax)
        h2_dot = -((qD_dot.T @ ax) + (ax_dot.T @ qD))/np.linalg.norm(ax)

        h3 = (qD.T @ ax)/np.linalg.norm(ax) - self.l_inner

        h3_dot = (qD_dot.T @ ax + ax_dot.T @ qD) / np.linalg.norm(ax)

        self._vals = [h1, h1_dot, h2, h2_dot, h3, h3_dot]

        return self._vals 
