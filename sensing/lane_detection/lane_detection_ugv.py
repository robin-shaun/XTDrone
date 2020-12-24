import numpy as np
import cv2
import math
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import sys
bridge = CvBridge()

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8
    )
    img = np.copy(img)
    if lines is None:
        return
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
    return img

def yellow_dectection(image):
    # create hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.uint8([ 30, 150, 101])
    upper = np.uint8([ 100, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image,mask=yellow_mask)

    return result   

def pipeline(image):
    image = yellow_dectection(image)
    height = image.shape[0]
    width = image.shape[1]

    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    cannyed_image = cv2.Canny(gray_image, 100, 200)
    
    lines = cv2.HoughLinesP(
        cannyed_image,
        rho=6,
        theta=np.pi / 60,
        threshold=200,
        lines=np.array([]),
        minLineLength=50,
        maxLineGap=50
    )  
    
    if lines is None:
        img_ros = bridge.cv2_to_imgmsg(image, "rgb8")
        return img_ros, 0
 
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []
 
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = float(y2 - y1) / float(x2 - x1)
            if slope <= 0:
                left_line_x.extend([x1, x2])
                left_line_y.extend([y1, y2])
            else:
                right_line_x.extend([x1, x2])
                right_line_y.extend([y1, y2])

    min_y = int(image.shape[0]* 1 / 2)
    max_y = int(image.shape[0]* 3 / 5)
    
    if not left_line_x == []:
        poly_left = np.poly1d(np.polyfit(left_line_y,left_line_x,deg=1))

        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))
    if not right_line_x == []:
        poly_right = np.poly1d(np.polyfit(right_line_y,right_line_x,deg=1))
        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))
    if not left_line_x == [] and not right_line_x == []:   
        mid_x_error = (left_x_end+right_x_end)/2.0-width/2
        line_image = draw_lines(image,[[[left_x_start, max_y, left_x_end, min_y],[right_x_start, max_y, right_x_end,min_y],]],thickness=5,)
        img_ros = bridge.cv2_to_imgmsg(line_image, "rgb8")
    else:
        mid_x_error = 1000
        img_ros = bridge.cv2_to_imgmsg(image, "rgb8")

    return img_ros, mid_x_error 
    

def img_callback(msg):
    global img_processed, lane_mid_error
    cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    img_processed, lane_mid_error = pipeline(cv_image)
    img_processed.header.stamp = rospy.Time.now()
    img_processed_pub.publish(img_processed)


if __name__ == "__main__":
    ugv_num = sys.argv[1]
    rospy.init_node("lane_detection_"+ugv_num)
    img_sub = rospy.Subscriber("/ugv_"+ugv_num+"/triclops/triclops/left/image", Image, callback=img_callback)
    img_processed_pub = rospy.Publisher("/ugv_"+ugv_num+"/image_lane", Image,queue_size=2)
    lane_mid_error_pub = rospy.Publisher("/ugv_"+ugv_num+"/lane_mid_error",Int16,queue_size=2)
    img_processed = Image()
    lane_mid_error = Int16()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        lane_mid_error_pub.publish(lane_mid_error)
        rate.sleep()
        
    