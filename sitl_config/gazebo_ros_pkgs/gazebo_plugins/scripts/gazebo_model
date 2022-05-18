#!/usr/bin/env python
# 
#  testing, do not use 
# 
# Provides quick access to the services exposed by gazebo_ros_factory plugin

import roslib, time
roslib.load_manifest('gazebo_plugins')

import rospy, sys
import string

from gazebo_plugins import gazebo_plugins_interface
from gazebo_plugins.msg import GazeboModel
from geometry_msgs.msg import Pose, Point, Quaternion

def usage():
    print '''Commands:
    load [param|file] [urdf|gazebo] <param/file name> <model name> <initial pose: x y z r p y> <namespace> - Load model
    delete <model name>                                                        - Deletes the model named <model name>
    '''
    sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) < 13:
        print usage()

    if sys.argv[1] == 'load':
        if sys.argv[2] == 'param':
            print "loading from parameter"
        elif sys.argv[2] == 'file':
            print "loading from file"
        else:
            print "invalid syntax, first argument has to be either param or file"
            print usage()

        if sys.argv[3] == 'urdf':
            print "loading urdf"
        elif sys.argv[3] == 'gazebo':
            print "loading gazebo model"
        else:
            print "invalid syntax, xml type has to be either urdf or gazebo"
            print usage()

        param_name = sys.argv[4]
        model_name = sys.argv[5]
        lx = float(sys.argv[6])
        ly = float(sys.argv[7])
        lz = float(sys.argv[8])

        #FIXME: rotation ignored for now
        ax = float(sys.argv[9])
        ay = float(sys.argv[10])
        az = float(sys.argv[11])

        namespace = sys.argv[12]

        model_msg = GazeboModel(model_name,param_name,GazeboModel.URDF_PARAM_NAME,namespace,Pose(Point(lx,ly,lz),Quaternion()))
        success = gazebo_plugins_interface.load_model(model_msg)
        print "spawning success",success

    elif sys.argv[1] == 'delete':
        print "not implemented yet"

    else:
        print "unknown command: ",sys.argv[1]
        print usage()

