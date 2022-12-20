#!/usr/bin/env python
# Initial Date: June 2020
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import roslaunch
import  time



""" This class is an utility class for creating and launching roslaunch, as well as terminating it """

'''
Summary of Class layout:
This class requires a ros package 'Sparkle'

Attributes:
    1. launchfile: The full path of the launch file
    2. theta: angular separation of two consecutive vehicle on the circle

    and all attributes of super class

Functions:
    1. __init__(circumference, n_vehicles): basically a constructor
'''
class launch:
    '''
    `launch`:A class facilitating roslaunch with runtime arguments and termination

    Parameters
    -------------

    launchfile: `string`
        Full path of the roslaunch file to be launch. Must be string. 

    kwargs
            variable keyword arguments passed as run-time argument for the launch file

    Attributes
    ------------
    launchfile: `string`
        Full path of the roslaunch file to be launch. Must be string. 

    runtime_args: `list`
        A list of run time arguments to be passed to launch file execution

    uuid: `string`
        Unique Identifier for the launch
    
    parent:  `roslaunch.parent.ROSLaunchParents`
        Length of car from bumper to bumper

    See Also
    ---------
    layout: superclass of `lane`

    '''
    def __init__(self, launchfile, **kwargs):
        self.launchfile = launchfile
        
        try:
            roslaunch.rlutil.resolve_launch_arguments([self.launchfile])    
        except roslaunch.RLException:
            print("Unable to find {}".format(self.launchfile))

        self.runtime_args = []
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        print(self.uuid)
        roslaunch.configure_logging(self.uuid)
        
        for key in kwargs.keys():
            self.runtime_args.append("{}:={}".format(key, kwargs[key]))

        if len(self.runtime_args) > 0:
            self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, [(self.launchfile, self.runtime_args)])
        else:
            self.parent = roslaunch.parent.ROSLaunchParent(self.uuid,[self.launchfile])
    def start(self):
        '''
        Calls `start()` function to execute roslaunch
        '''
        self.parent.start()
        time.sleep(5)
        if len(self.runtime_args) > 0:
            print("{} started with run-time arguments {}".format(self.launchfile, self.runtime_args))
        else:
            print("{} started.".format(self.launchfile))

    def shutdown(self):
        '''
        Calls `shutdown()` function to terminate the execution of the launch file
        '''
        self.parent.shutdown()
        print("{} Terminated.".format(self.launchfile))

