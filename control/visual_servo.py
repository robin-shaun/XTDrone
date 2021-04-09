import rospy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformListener, Buffer
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

class Tracker():
    def __init__(self, vehicle_type, vehicle_id):
        self.K = np.eye(3)
        self.E = np.hstack((np.eye(3),np.array([[0.1],[0.1],[0.5]])))
        self.lines_3d = np.array([[[0.05,0,0],[0.05,0.05,0]],[[0.05,0.05,0],[0.05,0.05,0.05]],[[0.05,0.05,0.05],[0.05,0,0.05]],[[0.05,0,0.05],[0.05,0,0]],[[0,0,0],[0,0.05,0]],[[0,0.05,0],[0,0.05,0.05]],[[0,0.05,0.05],[0,0,0.05]],[[0,0,0.05],[0,0,0]],[[0,0,0],[0.05,0,0]],[[0,0.05,0],[0.05,0.05,0]],[[0,0,0.05],[0.05,0,0.05]],[[0,0.05,0.05],[0.05,0.05,0.05]]])
        self.points_2d = []
        # self.plot_3d()
        rospy.init_node(vehicle_type+'_'+vehicle_id+'_visual_servo')
        rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/le_arm/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/le_arm/camera/camera_info', CameraInfo, self.camera_info_callback)
        self.image_pub = rospy.Publisher(vehicle_type+'_'+vehicle_id+'/le_arm/visual_servo/image', Image, queue_size=2)

        rate = rospy.Rate(20) 
        
        while not rospy.is_shutdown():
            rate.sleep()

    def project(self):
        for line_3d in self.lines_3d:
            for point_3d in line_3d:
                point_2d = self.K.dot(self.E).dot(np.vstack((point_3d.reshape(3,1),1)))
                point_2d = point_2d.astype(int)
                if self.points_2d == []: 
                    self.points_2d = point_2d
                else:
                    self.points_2d = np.hstack((self.points_2d, point_2d))
            

    def plot_3d(self):
        for line_3d in self.lines_3d:
            print(zip(line_3d[0],line_3d[1]))
            ax.plot(*zip(line_3d[0],line_3d[1]),color="b")
        plt.show()

    def draw_lines(self, img, points, color=[255, 0, 0], thickness=3):
        line_img = np.zeros(
            (
                img.shape[0],
                img.shape[1],
                3
            ),
            dtype=np.uint8
        )
        img = np.copy(img)
        if points is None:
            return
        for i in range(points.shape[1]/2):
            cv2.line(line_img, (points[0,2*i], points[1,i]), (points[0,i+1], points[1,2*i+1]), color, thickness)
        img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
        return img

    def camera_info_callback(self, msg):
        self.K = np.array(msg.K).reshape([3,3])
        print(self.K)

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
        self.project()
        line_image = self.draw_lines(cv_image,self.points_2d,thickness=5)

        img_ros = bridge.cv2_to_imgmsg(line_image)
        self.image_pub.publish(img_ros)
    

if __name__ == '__main__':
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    tracker = Tracker(vehicle_type, vehicle_id)

