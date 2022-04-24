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
def sgn(x):

    if x > 0:
        x = 1
    else:
        x = -1

    return x

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

    # Truth values
    theta_t = 0.1
    a       = 1

    # Estimate values
    theta = 0.2

    # Calculate control
    b0 = 0.1
    s  = a*x1 + x2
    v  = sgn(s)*(-theta*x1*x2 - b0)
    u  = -a*(x2+np.sin(x1)) + v

    # Calculate ODEs
    xd1 = x2 + np.sin(x1)
    xd2 = theta_t*x1*x2 + u

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
tf = 20.0
t = [t0, tf]

## Initial conditions
x0 = [0.2, 0.1]

# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, title="8", control=control, legend=['x1', 'x2'])
