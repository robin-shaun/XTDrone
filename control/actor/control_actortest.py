import rospy
import random
from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetModelState
import sys
import numpy

class ControlActor:
    def __init__(self, actor_id):
        self.count = 0
        self.f = 100
        self.flag = True
        self.distance_flag = False
        self.suitable_point = True
        self.x = 0.0
        self.y = 0.0
        self.x_max = 100.0
        self.x_min = -50.0
        self.y_max = 50.0
        self.y_min = -50.0
        self.id = actor_id
        self.velocity = 1.5
        self.last_pose = Point()
        self.current_pose = Point()
        self.target_pose = Point()
        self.target_pose.z = 1.25
        self.gazebo_actor_pose = Point()
        #self.black_box = numpy.array([[[-33, -20], [17, 33]], [[6, 19], [11, 27]], [[54, 67], [14, 30]], [[71, 83], [9, 19]], [[87, 101], [11, 17]], [[78, 95], [23, 34]], [[53, 70], [-35, -26]], [[-5, 5], [-34, -21]], [[13, 39], [-19, -9]], [[-6, 7], [-20, -10]], [[-28, -23], [-15, -28]], [[-36, -31], [-26, -13]], [[-37, -23], [-35, -30]]])
        self.cmd_pub = rospy.Publisher('/actor_' + str(self.id) + '/cmd_pose', Point, queue_size=10)
        self.black_box = numpy.array(
            [[[-32, -21], [18, 32]], [[7, 18], [12, 26]], [[55, 66], [15, 29]], [[72, 82], [10, 18]],
             [[88, 100], [12, 16]], [[79, 94], [24, 33]], [[54, 69], [-34, -27]], [[-4, 4], [-33, -22]],
             [[14, 38], [-18, -10]], [[-7, 6], [-19, -11]], [[-27, -24], [-14, -29]], [[-35, -32], [-25, -14]],
             [[-36, -24], [-34, -31]]])
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        print('actor_' + self.id + ": " + "communication initialized")

    def loop(self):
        rospy.init_node('actor_' + str(self.id))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            self.count = self.count + 1
            try:
                get_state = self.gazeboModelstate('actor_' + self.id, 'ground_plane')
                self.last_pose = self.current_pose
                self.gazebo_actor_pose = get_state.pose.position
                self.current_pose = self.gazebo_actor_pose
            except rospy.ServiceException, e:
                print "Gazebo model state service call failed: %s" % e
                self.current_pose.x = 0.0
                self.current_pose.y = 0.0
                self.current_pose.z = 1.25
            if self.flag or self.distance_flag:
                self.x = random.uniform(self.x_min, self.x_max)
                self.y = random.uniform(self.y_min, self.y_max)
                self.flag = False
                self.target_pose.x = self.x
                self.target_pose.y = self.y
                self.distance_flag = False
                self.suitable_point = True
            self.cmd_pub.publish(self.target_pose)
            if self.count % 20 == 0:
                print('current_pose' + self.id+':', self.current_pose)
                print('target_pose' + self.id+':', self.target_pose)
            distance = (self.current_pose.x-self.target_pose.x)**2+(self.current_pose.y-self.target_pose.y)**2
            if distance < 0.001:
                self.distance_flag = True
            if self.suitable_point:
                for i in range(13):
                    if (self.current_pose.x > self.black_box[i][0][0]) and (self.current_pose.x < self.black_box[i][0][1]):
                        if (self.current_pose.y > self.black_box[i][1][0]) and (self.current_pose.y < self.black_box[i][1][1]):
                            self.target_pose.x = self.current_pose.x + 100*(self.last_pose.x-self.current_pose.x)
                            self.target_pose.y = self.current_pose.y + 100*(self.last_pose.y-self.current_pose.y)
                            self.suitable_point = False
                            break
            rate.sleep()


if __name__=="__main__":
    controlactors = ControlActor(sys.argv[1])
    controlactors.loop()