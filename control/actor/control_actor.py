import rospy
import random
from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetModelState
import sys
import numpy

class ControlActor:
    def __init__(self, actor_id):
        self.f = 100
        self.x = [0.0 for i in range(5)]
        self.y = [0.0 for i in range(5)]
        self.x_max = 100.0
        self.x_min = -50.0
        self.y_max = 50.0
        self.y_min = -50.0
        self.id = actor_id
        self.target_pose = Point()
        self.target_pose.z = 1.25
        self.cmd_pub = rospy.Publisher('/actor_' + str(self.id) + '/cmd_pose', Point, queue_size=10)
        self.black_box = numpy.array([[[-34, -19], [16, 34]], [[5, 20], [10, 28]], [[53, 68], [13, 31]], [[70, 84], [8, 20]], [[86, 102], [10, 18]], [[77, 96], [22, 35]], [[52, 71], [-34, -25]], [[-6, 6], [-35, -20]], [[12, 40], [-20, -8]], [[-7, 8], [-21, -9]], [[-29, -22], [-16, -27]], [[-37, -30], [-27, -12]], [[-38, -24], [-36, -29]]])
        print('actor_' + self.id + ": " + "communication initialized")

    def loop(self):
        rospy.init_node('actor_' + str(self.id))
        rate = rospy.Rate(self.f)
        #target_pose needs to meet x_max,x_min,y_max,y_min and make sure that the target is not in the black box.
        self.x = [0.0, 2.0, 2.0, -2.0, -2.0]
        self.y = [0.0, 2.0, -2.0, 2.0, -2.0]
        for i in range(5):
            if self.x[i] > self.x_max:
                self.x[i] = self.x_max
            if self.x[i] < self.x_min:
                self.x[i] = self.x_min
            if self.y[i] > self.y_max:
                self.y[i] = self.y_max
            if self.y[i] < self.y_min:
                self.y[i] = self.y_min
        while not rospy.is_shutdown():
            self.target_pose.x = self.x[int(self.id)-1]
            self.target_pose.y = self.y[int(self.id)-1]
            self.cmd_pub.publish(self.target_pose)
            rate.sleep()

if __name__=="__main__":
    controlactors = ControlActor(sys.argv[1])
    controlactors.loop()