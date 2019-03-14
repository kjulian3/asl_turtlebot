#!/usr/bin/env python

import rospy
import os
# watch out on the order for the next two imports lol
import tf
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from asl_turtlebot.msg import DetectedObject
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import time
import std_msgs.msg

COCO_ITEMS = ['bottle', 'banana', 'apple', 'sandwich', 'orange',
              'broccoli', 'carrot', 'hot_dog', 'pizza', 'donut',
              'cake', 'dog']

class FoodLog:

    def __init__(self):
        rospy.init_node('city_food_logger', anonymous=True)
        self.tf_listener = tf.TransformListener()

        #setup publisher
        self.food_log_pub = rospy.Publisher('/food_locations', std_msgs.msg.String, queue_size=10)

        self.config_conf_threshold = 0.4
        # self.last_update_time = time.time()
        # raw_input("wait here")
        # Set initial location ("home")
        # self.tf_listener.waitForTransform("/map",'/velodyne', rospy.Time.now(),rospy.Duration(20.0))
        # (home_location_triad, home_rotation_triad) = self.tf_listener.lookupTransform("/map", '/velodyne', rospy.Time(0))
        self.x_home = 0.0 #home_location_triad[0]
        self.y_home = 0.0 #home_location_triad[1]

        #initialize log "entries"
        # self.x_coords = np.zeros((1))
        # self.y_coords = np.zeros((1))
        # self.max_confs = np.zeros((1))
        #self.max_confs[0] = 0.0
        self.index = dict()
        self.food_log = dict()
        self.food_log["home"] = ["home", 1, self.x_home, self.x_home, 1.0]
        #Add all possible food options
        for item in COCO_ITEMS:
            rospy.Subscriber('/detector/{}'.format(item),
                                DetectedObject,
                                self.evaluate_food_candidate,
                                queue_size = 1)
        # print self.food_log

    def evaluate_food_candidate(self, msg):
        #print "Evaluating food item"
        if msg.confidence >= self.config_conf_threshold:
            self.record_food_item(msg)

    def record_food_item(self,msg):
        name = msg.name
        dist = msg.distance
        th_l = msg.thetaleft
        th_r = msg.thetaright
        #print "here"
        if name in self.food_log:
            #print("here")
            i = self.food_log[name][1]
            new_item = False
        else:
            i = len(self.food_log) + 1
            new_item = True

        (translation, rotation) = self.tf_listener.lookupTransform("/map", '/velodyne', rospy.Time(0))
        Q = tf.transformations.euler_from_quaternion(rotation)[2]

        x_bot = translation[0]
        y_bot = translation[1]

        L = dist * (np.tan(th_l) + np.tan(th_r))
        Ay = np.sign(th_r - th_l) * L * 0.5
        Ax = dist
        th_m = np.arctan2(Ay, Ax)
        d_tilde = np.sqrt(Ay*Ay + Ax*Ax)
        z = Q - th_m

        x_item_coord = x_bot + d_tilde * np.cos(z)
        y_item_coord = y_bot + d_tilde * np.sin(z)

        if new_item:
            self.food_log[name] = [name, i, x_item_coord, y_item_coord, msg.confidence]
            # self.x_coords[i] = x_item_coord
            # self.y_coords[i] = y_item_coord
            # self.max_confs[i] = msg.confidence
        else:
            if msg.confidence >= self.food_log[name][4]:
                print "Updating confidence of " , name
                self.food_log[name] = [name, i, x_item_coord, y_item_coord, msg.confidence]
                self.last_update_time = time.time()


    def loop(self):
        print "-----------------------------"
        print "Last updated (ros time):" #, self.last_update_time
        print self.food_log
        print " ============================"
        # (translation, rotation) = self.tf_listener.lookupTransform("/map", '/velodyne', rospy.Time(0))
        # print "Velodyne location", translation

        t_str = ""

        for n in self.food_log:
            c_str = self.food_log[n]
            for i in range(len(c_str)):
                if i == (len(c_str)-1):
                    t_str += str(c_str[i]) + ";"
                else:
                    t_str += str(c_str[i]) + ","
        self.food_log_pub.publish(t_str[:-1])
        print "finished publishing!: ", t_str


    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__=='__main__':
    flog = FoodLog()
    flog.run()
