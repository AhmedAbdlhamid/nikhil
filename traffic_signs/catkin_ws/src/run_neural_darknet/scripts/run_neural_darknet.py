#!/usr/bin/env python

import datetime
import os
import cv2
import time
import rospy
import sys
import numpy as np
import time
from bolt_msgs.msg import Control
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox

import sys

sys.path.append('../neural_net/')

import const
from image_converter import ImageConverter
from drive_run import DriveRun
from config import Config
from image_process import ImageProcess

##################
# Yolo class name
TRAFFIC_SIGN_STOP = 'stop_sign'        
TRAFFIC_SIGN_PEDESTRAIN_WALK = 'pedestrain_walk'        
TRAFFIC_SIGN_PARKING = 'parking'        
TRAFFIC_SIGN_RIGHT_TURN = 'right_turn'        
TRAFFIC_SIGN_LEFT_TURN = 'left_turn'        
TRAFFIC_SIGN_YIELD = 'yield'        
TRAFFIC_SIGN_SPEED_10 = 'speed_10'   
TRAFFIC_SIGN_SPEED_15 = 'speed_15'   
TRAFFIC_SIGN_SPEED_20 = 'speed_20'   
TRAFFIC_SIGN_SPEED_25 = 'speed_25' 

MIN_BBOX_WIDTH = 50
MIN_BBOX_HEIGHT = 40

STOP_SIGN_PAUSE_SEC = 6    
STOP_SIGN_IGNORE_SEC = 8

class BBox:
    def __init__(self):
        self.class_name = None
        self.x1 = self.left = 0
        self.y1 = self.top = 0
        self.x2 = self.right = 0
        self.y2 = self.bottom = 0
        self.detected = False
  
    def width(self):
        if self.detected is False:
            return 0
        return self.x2 - self.x1
        
    def height(self):
        if self.detected is False:
            return 0
        return self.y2 - self.y1
        
        
class NeuralControlDarknet:
    def __init__(self, weight_file_name, default_speed):
        rospy.init_node('run_neural')
        self.ic = ImageConverter()
        self.image_process = ImageProcess()
        self.rate = rospy.Rate(10)
        self.drive= DriveRun(weight_file_name)
        rospy.Subscriber('/bolt/front_camera/image_raw', Image, self.img_controller_cb)
        self.image = None
        self.image_processed = False

        # darknet	
        self.bbox = BBox()
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.darknet_controller_cb)
        
        # vehicle's initial speed
        self.default_speed = default_speed
        self.brake = False


    def img_controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)

        cropped = img[const.CROP_Y1:const.CROP_Y2,
                      const.CROP_X1:const.CROP_X2]

        img = cv2.resize(cropped,(const.IMAGE_WIDTH, const.IMAGE_HEIGHT))

        self.image = self.image_process.process(img)
        self.image_processed = True


    def darknet_controller_cb(self, data):
        box_data = data.bounding_boxes

        self.bbox.class_name = box_data[0].Class
        self.bbox.x1 = box_data[0].xmin
        self.bbox.y1 = box_data[0].ymin
        self.bbox.x2 = box_data[0].xmax
        self.bbox.y2 = box_data[0].ymax

        self.bbox.detected = True

	
def main():

    if len(sys.argv) != 3:
        exit('Usage:\n$ rosrun run_neural_darknet run_neural_darknet.py weight_file_name default_speed(0~1)')

    # ready for neural network
    neural_control = NeuralControlDarknet(sys.argv[1], float(sys.argv[2]))
    print('\nStart running. Vroom. Vroom. Vroooooom......')

    # ready for /bolt topic publisher
    joy_pub = rospy.Publisher('/bolt', Control, queue_size = 10)
    joy_data = Control()

    # to measure time from the stopping
    time_s = None                 

    # After stopping, we have to ignore the stop sign detection for a while 
    time_r = None                 

    neural_control.brake = False  # no break

    while not rospy.is_shutdown():
        if neural_control.image_processed is False:
            continue

        if neural_control.brake is True:
            if int(time.time() - time_s) > STOP_SIGN_PAUSE_SEC:
                neural_control.brake = False
                joy_data.brake = 0
                joy_data.throttle = 1.0 # to restart from stopping
                joy_pub.publish(joy_data)
                time_r = time.time()
            else:
                joy_data.brake = 1.0
                joy_pub.publish(joy_data)
            continue

        # predicted steering angle from an input image
        prediction = neural_control.drive.run(neural_control.image)
        joy_data.steer = prediction

        # traffic sign detected? 
        if neural_control.bbox.width() > MIN_BBOX_WIDTH and \
                neural_control.bbox.height() > MIN_BBOX_HEIGHT:
            
            if neural_control.bbox.class_name == TRAFFIC_SIGN_STOP:
                # previously stop sign detected AND time hasn't passed to detect again
                if time_r is not None \
                    and int(time.time() - time_r) < STOP_SIGN_IGNORE_SEC:
                    continue

                # already brake is applied
                if neural_control.brake is True: 
                    # ignore the stop sign detection
                    continue 

                time_s = time.time() 
                neural_control.brake = True	 # brake applys
                print('Stop sign. ' + 'Wait for seconds...')
            elif neural_control.bbox.class_name == TRAFFIC_SIGN_SPEED_10:
                joy_data.throttle = 0.2
                joy_pub.publish(joy_data)
            elif neural_control.bbox.class_name == TRAFFIC_SIGN_SPEED_15:
                joy_data.throttle = 0.3
                joy_pub.publish(joy_data)
            elif neural_control.bbox.class_name == TRAFFIC_SIGN_SPEED_20:
                joy_data.throttle = 0.4
                joy_pub.publish(joy_data)
            elif neural_control.bbox.class_name == TRAFFIC_SIGN_SPEED_25:
                joy_data.throttle = 0.5
                joy_pub.publish(joy_data)

            print('Traffic sign detected: ' + neural_control.bbox.class_name)

        joy_data.throttle = neural_control.default_speed
        joy_pub.publish(joy_data)	     

        ## print out
        sys.stdout.write('steer: ' + str(joy_data.steer) +' throttle: ' + str(joy_data.throttle) + '\r')
        sys.stdout.flush()

        ## ready for processing a new input image
        neural_control.image_processed = False
        neural_control.bbox.detected = False

        neural_control.rate.sleep()


if __name__ == "__main__":
    try:
        main()
        
    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
