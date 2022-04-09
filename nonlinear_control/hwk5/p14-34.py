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
    x1, x2, x3, u = x

    # Calculate z
    z = x2

    # Calculate control
    u = -x1 + x2 + x1*x3 - z

    # Calculate ODEs
    xd1 = -x1 + x2
    xd2 = x1 - x2 - x1*x3 + u
    xd3 = x1 + x1*x2 - 2*x3

    # Bundle ODEs
    dx = [xd1, xd2, xd3, u]

    return dx

##==============================================================================
# Script

# Plotting/Calculating variables

## Time horizon
t0 = 0.0
tf = 8.0
t = [t0, tf]

## Initial conditions
x0 = [0.1, 0.2, 0.3, 0]


# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, tf=tf, title="14.34", legend=['x1', 'x2', 'x3', 'u'])
