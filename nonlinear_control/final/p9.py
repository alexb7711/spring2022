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
    ## Nominal controller
    k           = 5
    nom_control = x1*np.sin(x1) + theta*x1*x2 - k*x1 - k*x2

    ## Disturbance controller
    k0          = 0
    rho         = theta*x1*x2
    b0          = 0.5
    dis_control = (rho/(1-k0) + b0) * x2/np.linalg.norm(x2)

    ## Complete controller
    u           = nom_control - dis_control

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
tf = 5.0
t = [t0, tf]

## Initial conditions
x0 = [-3,2]

# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, title="9", control=control, legend=['x1', 'x2'])
