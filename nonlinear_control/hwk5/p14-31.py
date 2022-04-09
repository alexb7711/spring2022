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
    a      = 0

    # Calculate z
    z = x1 + x2 + a + (x1 - a**1/3)**3 + x1

    # Calculate control
    u = -x1 - x2 - 3*(x1**2)*x2 - z

    # Calculate ODEs

    xd1 = x2 + a + (x1 - a**1/3)**3
    xd2 = x1 + u

    # Bundle ODEs
    dx = [xd1, xd2, u]

    return dx

##==============================================================================
# Script

# Plotting/Calculating variables

## Time horizon
t0 = 0.0
tf = 8.0
t = [t0, tf]

## Initial conditions
x0 = [0.5, 0.6, 0]


# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, tf=tf, title="14.31", legend=['x1', 'x2', 'u'])
