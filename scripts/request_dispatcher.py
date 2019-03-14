#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
import tf
import numpy as np
from std_msgs.msg import String

use_gazebo = rospy.get_param("sim")
mapping = rospy.get_param("map")

# Thresholds for location
POS_EPS = .45
THETA_EPS = 2*np.pi

class RequestDispatcher:

    def __init__(self):
        #initialize node
        rospy.init_node('request_dispatcher', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.request = None
        self.request_received = False
        self.food_locations = {}
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.nav_pose_publisher = rospy.Publisher('/nav_pose', Pose2D, queue_size=10)
        # Pickup status for rviz
        self.delivery_status_publisher = rospy.Publisher('/delivery_status', String, queue_size=10)
        rospy.Subscriber('/delivery_request', String, self.process_request_callback)
        # TODO: check if this is the topic we're publishing to
        rospy.Subscriber('/food_locations', String, self.food_location_callback)

    def food_location_callback(self, msg):
        for item in msg.data.split(";"):
            fields = item.split(",")
            item_name = fields[0]
            item_id = float(fields[1])
            item_x_map = float(fields[2])
            item_y_map = float(fields[3])
            item_confidence = float(fields[4])
            self.food_locations[item_name] = (item_x_map, item_y_map)
        #rospy.loginfo("I now have:")
        #rospy.loginfo(str(self.food_locations))

    def process_request_callback(self, msg):
        """ callback for a pose goal sent through the request_publisher node """
        rospy.loginfo("Request received!")
        rospy.loginfo("Current known food locations:")
        rospy.loginfo(str(self.food_locations))
        if not self.request_received:
            # Reverse the order of the request so we can use pop to get
            # next item
            self.request = msg.data.split(",")
            self.request.append("home")
            self.request.reverse()
            self.request_received = True
            self.current_item = self.request.pop()
            rospy.loginfo("Received order:")
            rospy.loginfo("{}".format(str(self.request)))
            msg = "Picking up first item: {}".format(self.current_item)
            rospy.loginfo(msg)
            self.delivery_status_publisher.publish(msg)
        else:
            rospy.loginfo("Courier is already handling a request, try again later")

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """
        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS)# and abs(theta-self.theta)<THETA_EPS)

    def has_picked_up(self):
        return self.close_to(x=self.food_locations[self.current_item][0],
                             y=self.food_locations[self.current_item][1],
                             theta=0)

    def nav_to_current_item(self):
        if self.current_item in self.food_locations.keys():
            self.publish_goal_nav_pose(self.current_item)
        else:
            mg = "No location for {} is known. Looking for next item...".format(self.current_item)
            rospy.loginfo(msg)
            self.delivery_status_publisher.publish(msg)
            # Skip the current item
            self.current_item = None

    def process_request(self):
        self.nav_to_current_item()
        if self.current_item is not None:
            # If I am here, I know I have a current item and I know its location
            if self.has_picked_up() and len(self.request)>0:
                if len(self.request) == 1:
                    msg = "Picked up last item: {}! Going home.".format(self.current_item)
                    rospy.loginfo(msg)
                    self.delivery_status_publisher.publish(msg)
                    self.current_item = self.request.pop()
                else:
                    msg = "Picked up {}!".format(self.current_item)
                    rospy.loginfo(msg)
                    self.delivery_status_publisher.publish(msg)
                    self.current_item = self.request.pop()
                    msg = "Picking up {}".format(self.current_item)
                    rospy.loginfo(msg)
                    self.delivery_status_publisher.publish(msg)
            if self.has_picked_up() and len(self.request)==0:
                msg = "Finished processing the request! Ready to accept new ones"
                rospy.loginfo(msg)
                self.delivery_status_publisher.publish(msg)
                rospy.loginfo(msg)
                self.request_received = False
                self.current_item = None
        # If we need to get the next item
        else:
            if len(self.request)>0:
                self.current_item = self.request.pop()
                rospy.loginfo("Picking up {}".format(self.current_item))
            else:
                rospy.loginfo("Finished processing the request! Ready to accept new ones")
                self.request_received = False
                self.current_item = None


    def publish_goal_nav_pose(self, item):
        """ sends the current desired pose to the nav_pose topic """
        nav_pose_msg = Pose2D()
        nav_pose_msg.x = self.food_locations[item][0]
        nav_pose_msg.y = self.food_locations[item][1]
        nav_pose_msg.theta = self.theta
        self.nav_pose_publisher.publish(nav_pose_msg)
        # TODO: should I publish this more than once?

    def loop(self):
        # This is to get the current robot pose
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        else:
            rospy.loginfo("Oooops, you should be using Gazebo")

        # Actually do the request processing
        if self.request_received:
            self.process_request()

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        rospy.loginfo("Waiting for a new delivery request...")
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    dispatcher = RequestDispatcher()
    dispatcher.run()
