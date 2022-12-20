#!/usr/bin/env python


# Author : Rahul Bhadani, Eugene  Vinitsky
# Initial Date: Dec 2, 2020
# License: MIT License

#   Permission is hereby granted, free of charge, to any person obtaining
#   a copy of this software and associated documentation files
#   (the "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject
#   to the following conditions:

#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.

import math
import rospy

def safe_accel(accel, ego_speed, lead_speed, headway, sim_step, **kwargs):
    """Compute a safe accel for the vehicles.
    Checks if the accel will lead to a velocity such that if the lead vehicle were to stop
    instantaneously, we cannot bring the following vehicle to rest at the point at
    which the headway is zero. If so, clip the accel.
    WARNINGS:
    1. We assume the lead vehicle has the same deceleration capabilities as our vehicles
    2. We solve for this value using the discrete time approximation to the dynamics. We assume that the
       integration scheme induces positive error in the position, which leads to a slightly more conservative
       driving behavior than the continuous time approximation would induce. However, the continuous time
       safety rule would not be strictly safe.

    Parameters
    --------------
    accel : float
        current predicted acceleration input

    ego_speed: float
        Ego's current speed

    lead_speed: float
        Lead's current speed

    headway: float
        current headway distance

    sim_step: float
        Simulation Step size

    Returns
    -----------
    float
        maximum safe acceleration given a maximum deceleration, delay in
        performing the breaking action, and speed limit
    """
    max_decel = kwargs.get("max_decel", 4.6)
    lead_max_deaccel = kwargs.get("lead_max_deaccel", 4.6)
    min_gap = kwargs.get("min_gap",2.5)
    brake_dist = brake_distance(lead_speed, max(max_decel, lead_max_deaccel), 0, sim_step)
    v_safe = maximum_safe_stop_speed_euler(headway + brake_dist - min_gap, sim_step)
    if ego_speed > v_safe:
        rospy.loginfo(
            "=====================================\n"
            "Speed of vehicle is greater than safe speed. Safe velocity "
            "clipping applied.\n"
            "=====================================")
    if ego_speed + accel * sim_step > v_safe:
        if v_safe > 0:
            return (v_safe - ego_speed) / sim_step
        else:
            return -ego_speed / sim_step
    else:
        return accel
def brake_distance(speed, max_deaccel, delay, sim_step):
    """
    
    Return the distance needed to come to a full stop if braking as hard as possible.
    
    Parameters
    ----------
    speed : float
        ego speed
    max_deaccel : float
        maximum deaccel of the vehicle
    delay : float
        the delay before an action is executed
    is_ballistic : bool
        whether the integration stop is ballistic
    sim_step : float
        size of simulation step
    Returns
    -------
    float
        the distance required to stop
    """
    # how much we can reduce the speed in each timestep
    speedReduction = max_deaccel * sim_step
    # how many steps to get the speed to zero
    steps_to_zero = int(speed / speedReduction)
    return sim_step * (steps_to_zero * speed - speedReduction * steps_to_zero * (steps_to_zero + 1) / 2) + \
        speed * delay
def maximum_safe_stop_speed_euler(brake_dist, sim_step):
    """
    Compute the maximum speed that you can travel at and guarantee no collision for euler integration.
    Parameters
    ----------
    brake_dist : float
        total distance the vehicle has before it must be at a full stop
    sim_step : float
        simulation step size in seconds
    Returns
    -------
    v_safe : float
        maximum speed that can be travelled at without crashing
    """
    if brake_dist <= 0:
        return 0.0
    speed_reduction = 4.6 * sim_step
    s = sim_step
    t = 0
    # h = the distance that would be covered if it were possible to stop
    # exactly after gap and decelerate with max_deaccel every simulation step
    # h = 0.5 * n * (n-1) * b * s + n * b * t (solve for n)
    # n = ((1.0/2.0) - ((t + (pow(((s*s) + (4.0*((s*((2.0*h/b) - t)) + (t*t)))), (1.0/2.0))*sign/2.0))/s))
    sqrt_quantity = math.sqrt(
        ((s * s) + (4.0 * ((s * (2.0 * brake_dist / speed_reduction - t)) + (t * t))))) * -0.5
    n = math.floor(.5 - ((t + sqrt_quantity) / s))
    h = 0.5 * n * (n - 1) * speed_reduction * s + n * speed_reduction * t
    assert(h <= brake_dist + 1e-6)
    # compute the additional speed that must be used during deceleration to fix
    # the discrepancy between g and h
    r = (brake_dist - h) / (n * s + t)
    x = n * speed_reduction + r
    assert(x >= 0)
    return x