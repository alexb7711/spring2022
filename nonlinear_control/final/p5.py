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
    # Global variables
    global control

    # Extract states
    x1, x2 = x

    # Calculate z
    z1 = x1
    z2 = x2 + 1 + (1-x1)**3
    z  = np.array([[z1], [z2]])

    # Calculate control
    k = 10
    v = -k*x1 - k*x2
    u = x1 - 3*z2*(x1**2-3*x1-1) + v

    # Calculate ODEs
    xd1 = x2 + 1 + (1-x1)**3
    xd2 =  -x1 + u

    # Append control applied
    control = np.append(control,[u])

    # Bundle ODEs
    dx = [xd1, xd2]

    return dx

##==============================================================================
# Script

# Plotting/Calculating variables
control = np.array([])

## Time horizon
t0 = 0.0
tf = 15.0
t = [t0, tf]

## Initial conditions
x0 = [1, 0.1]

# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, control=control, title="5", legend=['x1', 'x2'])
