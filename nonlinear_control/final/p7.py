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
def saturate(x):

    if x > 1:
        x = 1
    elif x < -1:
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

    # Calculate control
    k = 1
    u = -k*x2

    # Calculate ODEs
    xd1 = x2
    xd2 =  -x1**3 + saturate(u)

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
tf = 100.0
t = [t0, tf]

## Initial conditions
x0 = [1, 0.5]

# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, control=control, title="7", legend=['x1', 'x2'])
