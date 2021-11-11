import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist, Pose, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformListener, Buffer
import sys
import cv2
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
bridge = CvBridge()
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

class Tracker():
    def __init__(self, vehicle_type, vehicle_id):
        self.K = []
        self.E = np.hstack((np.eye(3),np.array([[0.25],[0.25],[1]])))
        self.lines_3d = np.array([[[0.05,0,0],[0.05,0.05,0]],[[0.05,0.05,0],[0.05,0.05,0.05]],[[0.05,0.05,0.05],[0.05,0,0.05]],[[0.05,0,0.05],[0.05,0,0]],[[0,0,0],[0,0.05,0]],[[0,0.05,0],[0,0.05,0.05]],[[0,0.05,0.05],[0,0,0.05]],[[0,0,0.05],[0,0,0]],[[0,0,0],[0.05,0,0]],[[0,0.05,0],[0.05,0.05,0]],[[0,0,0.05],[0.05,0,0.05]],[[0,0.05,0.05],[0.05,0.05,0.05]]])
        self.lines_3d = self.lines_3d - np.array([0.02,0.02,0.02])
        self.points_2d = []
        self.box_pose = Pose()
        self.camera_pose = Pose()
        # self.plot_3d()
        rospy.init_node(vehicle_type+'_'+vehicle_id+'_visual_servo')
        rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/realsense/depth_camera/color/image_raw', Image, self.image_callback,queue_size=1)
        rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/realsense/depth_camera/color/camera_info', CameraInfo, self.camera_info_callback,queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback,queue_size=1)
        self.image_pub = rospy.Publisher(vehicle_type+'_'+vehicle_id+'/visual_servo/image', Image, queue_size=1)

        rate = rospy.Rate(20) 
        
        while not rospy.is_shutdown():
            rate.sleep()

    def project(self):
        for line_3d in self.lines_3d:
            for point_3d in line_3d:
                point_2d = self.K.dot(self.E).dot(np.vstack((point_3d.reshape(3,1),1)))
                if self.points_2d == []: 
                    self.points_2d = point_2d
                else:
                    self.points_2d = np.hstack((self.points_2d, point_2d))

    def plot_3d(self):
        for line_3d in self.lines_3d:
            ax.plot(*zip(line_3d[0],line_3d[1]),color="b")
        plt.show()

    def draw_lines(self, img, points, color=[255, 0, 0], thickness=3):
        line_img = np.zeros(
            (
                img.shape[0],
                img.shape[1],
                1
            ),
            dtype=np.uint8
        )
        img = np.copy(img)
        if points is None:
            return
        for i in range(points.shape[1]/2):
            cv2.line(line_img, (int(points[0,2*i]/points[2,2*i]), int(points[1,2*i]/points[2,2*i])), (int(points[0,2*i+1]/points[2,2*i+1]), int(points[1,2*i+1]/points[2,2*i+1])), color, thickness)
        img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
        return img
    
    def link_states_callback(self, msg):
        try:
            box_id = msg.name.index("box::base_link")
            camera_id = msg.name.index("iris_0::realsense_camera::link")
            box_pose = msg.pose[box_id]
            box_R = R.from_quat([box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w]).as_dcm()
            box_t = np.array([[box_pose.position.x],[box_pose.position.y],[box_pose.position.z]])
            box_pose = np.hstack((box_R,box_t))
            box_pose = np.vstack((box_pose,np.array([0,0,0,1])))
            camera_pose = msg.pose[camera_id]
            camera_R = R.from_quat([camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z, camera_pose.orientation.w]).as_dcm()
            camera_R = camera_R.dot(R.from_euler('z',-90,degrees=True).as_dcm()).dot(R.from_euler('x',-90,degrees=True).as_dcm())
            camera_t = np.array([[camera_pose.position.x],[camera_pose.position.y],[camera_pose.position.z]])
            camera_pose = np.hstack((camera_R,camera_t))
            camera_pose = np.vstack((camera_pose,np.array([0,0,0,1])))
            self.E = np.linalg.inv(camera_pose).dot(box_pose)[:3,:]
        except ValueError:
            pass 


    def camera_info_callback(self, msg):
        self.K = np.array(msg.K).reshape([3,3])
        # print(self.K)

    def image_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        cannyed_image = cv2.Canny(gray_image, 100, 200)
        # cv2.imshow("cannyed_image", cannyed_image)
        # cv2.waitKey(0)
        # lines = cv2.HoughLinesP(
        #     cannyed_image,
        #     rho=6,
        #     theta=np.pi / 180,
        #     threshold=50,
        #     lines=np.array([]),
        #     minLineLength=5,
        #     maxLineGap=50
        # )
        # if lines is None:
        #     return
        if not self.K == [] and self.points_2d == []:
            self.project()
        if not self.points_2d == []:
            line_image = self.draw_lines(cannyed_image,self.points_2d,thickness=5)
            img_ros = bridge.cv2_to_imgmsg(line_image)
            self.image_pub.publish(img_ros)
    

if __name__ == '__main__':
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    tracker = Tracker(vehicle_type, vehicle_id)

