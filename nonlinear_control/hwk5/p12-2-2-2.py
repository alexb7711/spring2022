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
    # Extract state
    x1, x2, x3 = x

    # Calculate control
    k = np.array([10, 10, 5])
    u = -k@x

    # Calculate ODEs
    xd1 = x1 + x2
    xd2 = x1*x2**2 - x1 + x3
    xd3 = u

    # Bundle ODEs
    dx = [xd1, xd2, xd3]

    return dx

##==============================================================================
# Script

# Plotting/Calculating variables

## Time horizon
t0 = 0.0
tf = 15.0
t = [t0, tf]

## Initial conditions
x0 = [0.5, 0.1, 0.1]


# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, tf=tf, title="12.2.2.2", legend=['x1', 'x2'])
