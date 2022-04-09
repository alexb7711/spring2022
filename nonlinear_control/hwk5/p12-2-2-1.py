#!/bin/python

##==============================================================================
# Libraries
import matplotlib.pyplot as plt
import numpy as np

from scipy.integrate import solve_ivp

from plot import plot

##==============================================================================
# Functions

##------------------------------------------------------------------------------
#
def model(t,x):
    """
    input
        t: time step
        x: state
    output
        dx: differential output of ode

    """
    # Extract states
    x1, x2, u = x

    # Calculate z
    z1 = x1
    z2 = x1 + x2
    z  = np.array([[z1], [z2]])

    # Calculate control
    k = np.array([10, 10])
    u = -3*(x1**3)*x2 + 2*x1 + x2 - k@z

    # Calculate ODEs
    xd1 = x1 + x2
    xd2 = 3*(x1**2)*x2 + x1 + u

    # Bundle ODEs
    dx = [xd1, xd2, u]

    return dx

##==============================================================================
# Script

# Plotting/Calculating variables

## Time horizon
t0 = 0.0
tf = 15.0
t = [t0, tf]

## Initial conditions
x0 = [1, 0.1, 0]


# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, tf=tf, title="12.2.2.1", legend=['x1', 'x2', 'u'])
