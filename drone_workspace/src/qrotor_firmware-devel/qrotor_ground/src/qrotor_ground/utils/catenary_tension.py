import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize
import math


def catenary_in_3d(x0, x1, l, p):
    """
    Function that takes in two R3 position vectors (1x3 arrays) x0, x1, length of the rope l, and density of the rope p
    Returns two tension vectors in the global frame
    >>> cat_3d([0, 0, 0], [1, 0, 1], 1.9, 10/1.9) #2D example
    Catenary Parameter:  0.28144459433153257
    ([-14.531428791541064, 0.0, 21.71211184169785], [14.53142879154107, 0.0, 76.38788815830216])
    >>> cat_3d([0, 0, 0], [1, 1, 1], 2.5, 10/1.9)
    Catenary Parameter:  0.3967343290580736
    ([-14.484389329085946, -14.484389329085946, 37.21955684067066], [14.48438932908595, 14.48438932908595, 91.85939052775039])
    """
    # first, define h and d based on the inputs
    # defines the distance d based on the XY norm
    d = math.sqrt((x0[0]-x1[0])**2+(x0[1]-x1[1])**2)
    h = x1[2]-x0[2]  # defines the height based on the Z coord

    # define a convenient root finding function
    def fzero(func):
        return optimize.root_scalar(func, bracket=[0.001, 10], method='brentq')

    # solve for the tension vectors in the catenary frame
    def catenaryFunc(h, d, l, m):
        """
        Function to compute catenary profile and tension between two points
        Takes in two HORIZONTAL arrays
        catenaryFunc(1, 1, 1.9, 10)
        >>> tension vectors in catenary frame
        """
        # solve for the catenary parameter a
        def catenary_finder():
            def root_func(a):
                return 2*a*math.sinh(abs(d)/(2*a))-(l**2-h**2)**0.5
            return fzero(root_func)  # find the root of root_func
        a = catenary_finder().root
        print("Catenary Parameter: ", a)

        # solve for the bounds and positions of quadrotors in catenary frame
        # rebind fzero to fsolve
        def bound_finder():
            def bound_func(x):
                return a*math.cosh((x+d)/a)-a*math.cosh(x/a)-h
            l_bound = optimize.root_scalar(
                bound_func, bracket=[-1*l, l], method='brentq').root
            #l_bound = optimize.fsolve(bound_func, 0)
            return [l_bound, l_bound+d]
        # define the bounds
        bound = bound_finder()

        # define convenient tension variables
        x_0 = bound[0]
        x_1 = bound[1]

        # first, define slope unit direction vectors
        r0 = [1/math.sqrt((math.sinh(x_0/a))**2+1), 0,
              math.sinh(x_0/a)/math.sqrt((math.sinh(x_0/a))**2+1)]
        r1 = [1/math.sqrt((math.sinh(x_1/a))**2+1), 0,
              math.sinh(x_1/a)/math.sqrt((math.sinh(x_1/a))**2+1)]

        # solve linear system for tensions
        m_list = [[r0[0], r1[0]], [r0[2], r1[2]]]
        a = np.array(m_list)
        b = np.array([0, m*9.81])
        t = np.linalg.inv(a).dot(b)
        # print(t)

        # get tension vectors
        t0 = np.dot(r0, t[0])
        t1 = np.dot(r1, t[1])
        # print(t0)
        # print(t1)

        # return tension vectors
        return t0, t1
    # define the catenary tension vectors (Note p*l for mass m)
    t0_C, t1_C = catenaryFunc(h, d, l, p*l)
    # now, convert these back into the world frame using the rotation matrix around z axis (assumes only z rotations)

    def globalizer(xB, theta):
        # takes an R3 array rotated around the z in a "body" frame and converts it to the global frame
        xA = [0, 0, 0]
        xA[0] = math.cos(theta)*xB[0]-math.sin(theta)*xB[1]
        xA[1] = math.sin(theta)*xB[0]+math.cos(theta)*xB[1]
        xA[2] = xB[2]
        return xA
    # gets the angle of the catenary frame rel. to the global frame
    theta = math.atan((x1[1]-x0[1])/(x1[0]-x0[0]))
    t0 = globalizer(t0_C, theta)
    t1 = globalizer(t1_C, theta)
    # print(t0)
    # print(t1)
    return t0, t1  # return the tension vectors in the global frame
