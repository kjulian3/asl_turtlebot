#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, PoseArray, Pose2D, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import String
import tf
from copy import deepcopy
import numpy as np

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

class Turtlebot_Marker:
	def __init__(self):
		rospy.init_node('pub_marker')
		self.turtlebot_pub = rospy.Publisher("markers_turtlebot", MarkerArray, queue_size=10)
		self.goal_pub = rospy.Publisher("markers_goal", MarkerArray, queue_size=10)
		self.goal_pose  = None
		self.goal_pose_arrow = None
		self.trans_listener = tf.TransformListener()
		#rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
		rospy.Subscriber('/cmd_nav', Pose2D, self.goal_callback)

		self.food_pub = rospy.Publisher("markers_food", MarkerArray, queue_size=10)
		rospy.Subscriber('/food_locations', String, self.food_callback)

		self.status_pub = rospy.Publisher("markers_status", Marker, queue_size=10)
		rospy.Subscriber('/delivery_status', String, self.delivery_callback)

	def delivery_callback(self, msg):
		status = msg.data
		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time(0)
		marker.ns = "status"
		marker.action=Marker.ADD
		marker.type=9
		marker.scale.z = .2
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = -1.0
		marker.pose.position.z = 0.05
		marker.lifetime = rospy.Duration(600)
		marker.text=status
		self.status_pub.publish(marker)




	def food_callback(self,msg):
		food_list = msg.data.split(';')
		markerArray = MarkerArray()
		for food in food_list:
			food_items = food.split(',')
			name   = food_items[0]
			id_num = int(food_items[1])
			x      = float(food_items[2])
			y      = float(food_items[3])
			conf   = float(food_items[4])

			marker = Marker()
			marker.header.frame_id = "map"
			marker.header.stamp = rospy.Time(0)
			marker.ns = name+"_mark"
			marker.action=Marker.ADD
			marker.type=3
			marker.scale.x = .15
			marker.scale.y = .15
			marker.scale.z = .01
			marker.color.r = 1.0
			marker.color.g = 165.0/255.0
			marker.color.b = 0.0
			marker.color.a = 0.8
			marker.pose.position.x = x
			marker.pose.position.y = y
			marker.pose.position.z = 0.05
			marker.lifetime = rospy.Duration(20)
			markerArray.markers.append(marker)

			marker = Marker()
			marker.header.frame_id = "map"
			marker.header.stamp = rospy.Time(0)
			marker.ns = name+"_text"
			marker.action=Marker.ADD
			marker.type=9
			marker.scale.z = .2
			marker.color.r = 0.5
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.color.a = 1.0
			marker.pose.position.x = x
			marker.pose.position.y = y
			marker.pose.position.z = 0.05
			marker.lifetime = rospy.Duration(20)
			marker.text=name
			markerArray.markers.append(marker)
		if len(markerArray.markers)==0: return
		self.food_pub.publish(markerArray)


	def pose_markers(self):

		if self.goal_pose is not None:
			goal_marker_array = MarkerArray()
			origin_frame = "/map" if mapping else "/odom"
			goal_marker = Marker()
			goal_marker.header.frame_id = origin_frame
			goal_marker.header.stamp = rospy.Time(0)
			goal_marker.ns = "goal_circle"
			goal_marker.action = Marker.ADD
			goal_marker.pose = self.goal_pose
			goal_marker.type=3  
			goal_marker.scale.x = .15
			goal_marker.scale.y = .15
			goal_marker.scale.z = .01
			goal_marker.color.r = 0.0
			goal_marker.color.g = 1.0
			goal_marker.color.b = 0.0
			goal_marker.color.a = 0.5
			goal_marker.lifetime = rospy.Duration(600)
			goal_marker_array.markers.append(goal_marker)

			goal_marker_arrow = Marker()
			goal_marker_arrow.header.frame_id = origin_frame
			goal_marker_arrow.header.stamp = rospy.Time(0)
			goal_marker_arrow.ns = "goal_arrow"
			goal_marker_arrow.action = Marker.ADD
			goal_marker_arrow.pose = self.goal_pose_arrow
			goal_marker_arrow.type=0 
			goal_marker_arrow.scale.x = .25
			goal_marker_arrow.scale.y = .05
			goal_marker_arrow.scale.z = .05
			goal_marker_arrow.color.r = 0.0
			goal_marker_arrow.color.g = 1.0
			goal_marker_arrow.color.b = 0.0
			goal_marker_arrow.color.a = 0.5
			goal_marker_arrow.lifetime = rospy.Duration(600)
			goal_marker_array.markers.append(goal_marker_arrow)

			self.goal_pub.publish(goal_marker_array)

		markerArray = MarkerArray()
		slam_marker = Marker()
		slam_marker.header.frame_id = "/base_footprint"
		slam_marker.header.stamp = rospy.Time(0)
		slam_marker.ns = "turtlebot_circle"
		slam_marker.action=Marker.ADD
		slam_marker.type = 3    
		slam_marker.scale.x = .15
		slam_marker.scale.y = .15
		slam_marker.scale.z = .01
		slam_marker.color.r = 0.0
		slam_marker.color.g = 1.0
		slam_marker.color.b = 1.0
		slam_marker.color.a = 1.0
		slam_marker.lifetime = rospy.Duration(20)
		markerArray.markers.append(slam_marker)

		arrow_marker = Marker()
		arrow_marker.header.frame_id = "/base_footprint"
		arrow_marker.header.stamp = rospy.Time(0)
		arrow_marker.ns = "turtlebot_arrow"
		arrow_marker.action=Marker.ADD
		arrow_marker.type = 0    # line list
		arrow_marker.pose = Pose(Point(0.075, 0.0, 0.01), Quaternion(0, 0, 0, 1))
		arrow_marker.scale.x = .25
		arrow_marker.scale.y = .05
		arrow_marker.scale.z = .05
		arrow_marker.color.r = 0.0
		arrow_marker.color.g = 0.0
		arrow_marker.color.b = 1.0
		arrow_marker.color.a = 1.0
		arrow_marker.lifetime = rospy.Duration(20)
		markerArray.markers.append(arrow_marker)

		self.turtlebot_pub.publish(markerArray)


	def goal_callback(self, msg):

		self.goal_pose = Pose()
		self.goal_pose.position.x = msg.x
		self.goal_pose.position.y = msg.y
		theta = msg.theta
		quat = tf.transformations.quaternion_from_euler(0,0,theta)
		self.goal_pose.orientation.x = quat[0]
		self.goal_pose.orientation.y = quat[1]
		self.goal_pose.orientation.z = quat[2]
		self.goal_pose.orientation.w = quat[3]

		self.goal_pose_arrow = deepcopy(self.goal_pose)
		theta = msg.theta

		self.goal_pose_arrow.position.x += 0.075*np.cos(theta)
		self.goal_pose_arrow.position.y += 0.075*np.sin(theta)

	def goal_callback_old(self, msg):
		""" callback for a pose goal sent through rviz """
		origin_frame = "/map" if mapping else "/odom"
		try:
			self.goal_pose = self.trans_listener.transformPose(origin_frame, msg).pose
			self.goal_pose_arrow = deepcopy(self.goal_pose)

			quaternion = (
					self.goal_pose_arrow.orientation.x,
					self.goal_pose_arrow.orientation.y,
					self.goal_pose_arrow.orientation.z,
					self.goal_pose_arrow.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			theta = euler[2]
			self.goal_pose_arrow.position.x += 0.075*np.cos(theta)
			self.goal_pose_arrow.position.y += 0.075*np.sin(theta)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
	'''
	def state_callback(self, msg):
		self.ground_truth_ct = self.ground_truth_ct + 1
		# `rostopic hz /gazebo/model_states` = 1000; let's broadcast the transform at 20Hz to reduce lag
		if self.ground_truth_ct % 50 == 0:
			self.latest_pose_time = rospy.Time.now()
			self.latest_pose = msg.pose[msg.name.index("turtlebot3_burger")]
			self.tfBroadcaster.sendTransform(create_transform_msg(
				(self.latest_pose.position.x, self.latest_pose.position.y, 0),
				(self.latest_pose.orientation.x, self.latest_pose.orientation.y, self.latest_pose.orientation.z, self.latest_pose.orientation.w),
				"base_footprint", "world", self.latest_pose_time)
	'''
	def run(self):
		while not rospy.is_shutdown():
			rospy.sleep(0.01)
			self.pose_markers()


if __name__ == '__main__':
	marker = Turtlebot_Marker()
	marker.run()