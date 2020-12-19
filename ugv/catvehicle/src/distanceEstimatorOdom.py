#!/usr/bin/env python
# Taken from http://answers.ros.org/question/203270/how-do-i-get-the-distancies-between-frames-in-openni_tracker/
# Otherwise, author Jonathan Sprinkle
# Copyright (c) 2016 Arizona Board of Regents
import rospy
import tf
from std_msgs.msg import Float32
import geometry_msgs.msg
from numpy import (array, dot, arccos,arctan2)
from numpy.linalg import norm

if __name__ == '__main__':
    rospy.init_node('distanceEstimatorOdom')

    # get the name of the follower vehicle
    follower = rospy.get_param("~follower","catvehicle")
    leader = rospy.get_param("~leader","car1")
    disttopicname = rospy.get_param("~dist_topic","/distanceEstimator/dist")
    angletopicname = rospy.get_param("~angle_topic","/distanceEstimator/angle")

    # get the name of the leader vehicle

    listener = tf.TransformListener()
    publisherDist = rospy.Publisher(disttopicname, Float32, queue_size=1)
    publisherAngle = rospy.Publisher(angletopicname, Float32, queue_size=1)
    rate = rospy.Rate(75.0)
    while not rospy.is_shutdown():
        try:
#            t_tmp = listener.getLatestCommonTime('{0}/front_laser_link'.format(follower), '{0}/back_left_marker_link'.format(leader))
#            rospy.loginfo("t_tmp={0}".format(t_tmp))
            (trans,rot) = listener.lookupTransform('{0}/front_laser_link'.format(follower), '{0}/back_left_marker_link'.format(leader), rospy.Time())
            dist=norm(trans[0:1])
            rospy.logdebug("Translation: {0}, Rotation: {1}".format(trans,rot))
            # use arctan
            angle = arctan2(trans[1],trans[0])

            rospy.logdebug("Distance, angle between the points is = ({0:f},{1:f})".format(dist,angle))
            publisherDist.publish(dist)
            publisherAngle.publish(angle)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exceptMsg:
#            rospy.logerr("Unable to look up transformation from {0}/front_laser_link to {1}/back_left_marker_link: {2} (if you receive this during startup, it is not an error but just waiting for all tfs to come online)".format(follower,leader,exceptMsg))
            continue

        rate.sleep()

