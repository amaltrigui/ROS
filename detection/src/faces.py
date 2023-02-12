#!/usr/bin/env python3

# Python dependencies
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError

# ROS dependencies
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, LaserScan

bridge = CvBridge()
scan_steps = 902 #as mentioned in the provided vrep script
pub = rospy.Publisher('visualization_marker', Marker, queue_size=100) #default topic 'visualization_marker'
depth = np.zeros((scan_steps, ), dtype=np.float32)

def mark(x,y,w,h):
    m = Marker()
    m.header.frame_id = "laser_link"
    m.type = m.SPHERE # marker is of sphere shape
    #set the color of the marker to cyan
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 1.0
    #set the scale of the marker 
    m.scale.x = 0.4
    m.scale.y = 0.4
    m.scale.z = 0.4
    #display the marker forever
    m.lifetime = rospy.Duration(0)  
    #set other parameters of the marker                  
    m.action = m.ADD
    m.pose.orientation.w = 1.0
    m.id = 0
 
    #calculate the angle of the laser
    x_offset = x + w // 2 - 256
    angle = x_offset / 256.0 * (math.pi / 8)
    #get the scan distance 
    index = int((angle + math.pi) * scan_steps / math.pi / 2)
    scan_dist = depth[index]
    #calculate the marker position in the rviz map
    m.pose.position.x = math.sin(angle) * scan_dist
    m.pose.position.y = -math.cos(angle) * scan_dist
    m.pose.position.z = 0

    '''
    #calculate the angle of the laser
    center = image_x + w//2;
    angle = 67.5 + center / 512;
    #get the scan distance 
    index = int(902*angle/180)
    scan_dist = depth[index]
    #calculate the marker position in the rviz map
    m.pose.position.x = scan_dist * cos((degree/180)*math.pi)
    m.pose.position.y = scan_dist * sin((degree/180)*math.pi)
    m.pose.position.z = 0
    '''
    pub.publish(m)

def get_depth(msg):
    depth[:] = msg.ranges

def draw_marker(msg):
    image = None
    try:
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # declare an object of CascadeClassifier
    cascade = cv2.CascadeClassifier('/home/amal/project_ws/src/detection/src/haarcascade_frontalface_default.xml')
    # Convert into grayscale
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Detect faces
    faces = cascade.detectMultiScale(image_gray, 2, 4)
    # Draw rectangle around the faces
    for (x, y, w, h) in faces:
    	cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 3)
    	mark(x, y, w, h)
    # Display the output
    cv2.imshow('marked', cv2.flip(image, 1))
    cv2.waitKey(1)


def main():
    rospy.init_node('detection')
    image_topic = ""
    rospy.Subscriber('/vrep/image', Image, draw_marker)
    rospy.Subscriber('/vrep/scan', LaserScan, get_depth)
    rospy.spin()

if __name__ == '__main__':
    main()
