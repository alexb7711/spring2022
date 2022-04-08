#!/bin/python

##==============================================================================
# Libraries
import matplotlib.pyplot as plt
import numpy as np


##==============================================================================
#
def plot(solution,
         tf        : float    = 15,
         x_lbl     : str      = "t",
         legend    : np.array = ['x','y'],
         title     : str      = "Model",
         show_plot : bool     = False):
    """
    Plotting module for ODE's.

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

    # Determine whether to display or save plot
    if show_plot:
        plt.show()
    else:
        plt.savefig("img/"+title+".png")

    return
