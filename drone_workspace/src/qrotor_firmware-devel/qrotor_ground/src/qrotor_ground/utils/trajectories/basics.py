#!/usr/bin/env pythono

import sys
# import IPython
import numpy as np
import scipy as sp
import math

PI = sp.pi

def setpoint(t, sp=np.array([0., 0., 1.0])):

    traj = dict()
    traj['x'] = sp
    traj['dx'] = np.zeros(3)
    traj['d2x'] = np.zeros(3)
    traj['d3x'] = np.zeros(3)
    traj['d4x'] = np.zeros(3)
    traj['d5x'] = np.zeros(3)
    traj['d6x'] = np.zeros(3)

    return traj


def circleXY(t, r=1, c=np.zeros(3), w=0.1*PI):

    traj = dict()
    traj['x'] = c + r*np.array([math.cos(w*t), math.sin(w*t), 0])
    traj['dx'] = r*np.array([-1*w*math.sin(w*t), w*math.cos(w*t), 0])
    traj['d2x'] = r*np.array([-1*w**2*math.cos(w*t), -1*w**2*math.sin(w*t), 0])
    traj['d3x'] = r*np.array([w**3*math.sin(w*t), -1*w**3*math.cos(w*t), 0])
    traj['d4x'] =  r*np.array([w**4*math.cos(w*t), w**4*math.sin(w*t), 0])

    return traj