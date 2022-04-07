#!/bin/python

##==============================================================================
# Libraries
import matplotlib.pyplot as plt
import numpy as np

from scipy.integrate import solve_ivp

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
    x1, x2 = x

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
    dx = [xd1, xd2]

    return dx

##------------------------------------------------------------------------------
#
def plot(solution,
         tf     : float    = 15,
         x_lbl  : str      = "t",
         legend : np.array = ['x','y'],
         title  : str      = "Model"):
    """
    input
        solution: Solution from ode

    output
        NONE
    """

    # Extract
    t = np.linspace(0,tf,300)
    z = solution.sol(t)

    # Plot
    plt.plot(t, z.T)
    plt.xlabel(x_lbl)
    plt.legend(legend, shadow=True)
    plt.title(title)
    plt.show()
    return

##==============================================================================
# Script

# Plotting/Calculating variables

## Time horizon
t0 = 0.0
tf = 15.0
t = [t0, tf]

## Initial conditions
x0 = [1, 0.1]


# Solve ode model
sol = solve_ivp(model, t, x0, method='RK45', dense_output=True)

# Plot solution
plot(sol, tf=tf, title="12.2.2.1", legend=['x1', 'x2'])
