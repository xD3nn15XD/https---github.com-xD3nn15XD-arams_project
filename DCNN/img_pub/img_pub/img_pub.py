#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
# Imports the ROS message String from the ROS package std_msgs
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from custom_msg.srv import DetectionSrv
from std_msgs.msg import String, Bool

import cv2
from cv_bridge import CvBridge



def cam_sub_callback(msg):
    global sensor_img
    sensor_img = msg

def flag_sub_callback(msg):
    global flag
    flag = msg.data

# define callback function
def pub_callback():
    global pub, sensor_img, flag
    if flag:
        pub.publish(sensor_img)
    else:
        pass

def det_sub_callback(msgs):
    global detection, score
    
    i = 0
    while i < len(msgs.detections):
        j = 0
        while j < len(msgs.detections[i].results):
            if score < msgs.detections[i].results[j].score:
                detection = msgs.detections[i].results[j].id
                score = msgs.detections[i].results[j].score
            j = j +1

        i = i +1

    print('detected Img: ' + str(detection) + ' (score = ' + str(score) + ')')
           
def call_detection_callback(request, response):
    global detection, score, flag

    pub_callback()

    response.detection = detection
    
    print('Incoming request.')
    print(['Detection: ', detection])

    detection = ''
    score = 0.0
    flag = False

    return response

def main():
    global pub, sensor_img, detection, score, flag

    sensor_img = Image()
    detection = ""
    score = 0.0
    flag = False

    # initialize node
    rclpy.init()
    node_handle = rclpy.create_node('detector_conector')
    # initialize subscribers
    node_handle.create_subscription(Image, '/camera/image_raw', cam_sub_callback , 10)
    node_handle.create_subscription(Bool, 'detect', flag_sub_callback , 10)
    # initialize publisher AND declare timer with a callback every 3 seconds
    pub = node_handle.create_publisher(Image, '/detector_node/images', 1)
    node_handle.create_timer(2, pub_callback)
    # initialize subscribtion for detection
    node_handle.create_subscription(Detection2DArray, '/detector_node/detections', det_sub_callback , 10)
    # initialize Service Server
    node_handle.create_service(DetectionSrv, 'call_detection', call_detection_callback)
    
    try:
        rclpy.spin(node_handle)
    except KeyboardInterrupt:
        pass
    
    node_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()