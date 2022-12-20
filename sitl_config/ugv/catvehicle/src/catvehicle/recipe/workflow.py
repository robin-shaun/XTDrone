#!/usr/bin/env python
# Initial Date: November 2020
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" Predefines Workflow for simulation. """

from ..layout import layout
import time

def a_pair_of_catvehicles(t = 20):
    """
    Simply create a simulation with a pair of cars and terminates after t seconds

    Parameters
    -------------
    t: `float`
        Time in seconds till which simulations last.
    """
    
    ## Co-ordinates of catvehicles
    X = [0.0, 20.0]
    Y = [0.0, 0.0]
    Yaw = [0.0, 0.0]


    L = layout(n_vehicles= 2, X = X, Y = Y, Yaw=Yaw, description = "A test simulation with two catvehicles")
    L.create()
    L.spawn(include_laser = "all")
    L.rviz()
    time.sleep(t)
    L.destroy()


