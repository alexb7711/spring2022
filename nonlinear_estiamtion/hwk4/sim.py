#!/bin/python

import numpy as np

from math            import cos, sin, tan, exp, pi, atan
from scipy.integrate import odeint

def f(y,t):
    x1, x2, x3, = y

    z1 = sin(x3)
    z2 = tan(x2)
    z  = np.array([[z1], [z2]])

    A = np.array([[0,1],[0,0]])
    B = np.array([[0],[1]])
    gamma = cos(x2)**3*cos(x3)
    alpha = tan(x2)

    u = atan((-z1 -z2 - alpha)/gamma)

    print(A.shape)
    input(z.shape)

    return A@z

t = np.arange(0, 5, 0.1)
y0 = [0.1,0.1,0.1]
y = odeint(f,y0, t)
