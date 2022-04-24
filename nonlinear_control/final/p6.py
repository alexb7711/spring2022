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

    # Calculate control
    xd1 = x1**2 + x2

    phi  = -x1**2 - x1
    dphi = -2*x1*xd1 - xd1
    k    = -10
    z    = x2 - phi
    u    = dphi + k*z

    # Calculate ODEs
    ## xd1 is used for the control
    xd2 = u

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
plot(sol, control=control, title="6", legend=['x1', 'x2'])
