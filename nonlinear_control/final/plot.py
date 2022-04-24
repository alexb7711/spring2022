#!/bin/python

##==============================================================================
# Libraries
import matplotlib.pyplot as plt
import numpy as np


##==============================================================================
#
def plot(solution,
         x_lbl     : str      = "t",
         control   : np.array = np.array([]),
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
    t = solution.t
    z = solution.sol(t)

    # Check if there is a control to plot
    if control.size > 0:
        ## Update legend
        legend = np.append(legend, ['u'])

        ## Extract desired plot points
        u = np.array([[control[int(i)] for i in solution.t]])

        ## Add points to the list to be plotted
        z = np.append(z, u, axis=0)

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
