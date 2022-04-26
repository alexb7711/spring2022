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

    # Truth values
    a = 1

    # Calculate control
    k     = 1
    b     = 1
    gamma = 1
    r     = np.sin(t)
    rdot  = np.cos(t)
    e     = x - r
    ahat  = gamma*e*x
    u     = -1/b*(k*e - rdot + ahat*x)

    # Calculate ODEs
    xd = a*x + u

    # Append control applied
    control = np.append(control,[u])

    # Bundle ODEs
    dx = xd

    return dx

##==============================================================================
# Script

# Plotting/Calculating variables
control = np.array([])

## Time horizon
t0 = 0.0
tf = 5.0
t = [t0, tf]

## Initial conditions
x0 = [-3,2]

# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, title="10", control=control, legend=['x1'])
