#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseArray, Pose2D, Pose, Point, Quaternion, PoseStamped
import tf
from copy import deepcopy
import numpy as np

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

class Turtlebot_Marker:
	def __init__(self):
		rospy.init_node('pub_marker')
		self.circle_pub = rospy.Publisher("turtlebot_circle", Marker, queue_size=10)
		self.goal_circle_pub = rospy.Publisher("turtlebot_goal_circle", Marker, queue_size=10)
		self.arrow_pub = rospy.Publisher("turtlebot_arrow", Marker, queue_size=10)
		self.goal_arrow_pub = rospy.Publisher("turtlebot_goal_arrow", Marker, queue_size=10)
		self.goal_pose  = None
		self.goal_pose_arrow = None
		self.trans_listener = tf.TransformListener()
		rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

	def make_marker(self):
		self.slam_marker = Marker()
		self.slam_marker.header.frame_id = "/base_footprint"
		self.slam_marker.header.stamp = rospy.Time(0)
		self.slam_marker.ns = "turtlebot_circle"
		self.slam_marker.action=Marker.ADD
		self.slam_marker.type = 3    
		self.slam_marker.scale.x = .15
		self.slam_marker.scale.y = .15
		self.slam_marker.scale.z = .01
		self.slam_marker.color.r = 0.0
		self.slam_marker.color.g = 1.0
		self.slam_marker.color.b = 1.0
		self.slam_marker.color.a = 1.0
		self.slam_marker.lifetime = rospy.Duration(20)

		self.arrow_marker = Marker()
		self.arrow_marker.header.frame_id = "/base_footprint"
		self.arrow_marker.header.stamp = rospy.Time(0)
		self.arrow_marker.ns = "turtlebot_arrow"
		self.arrow_marker.action=Marker.ADD
		self.arrow_marker.type = 0    # line list
		self.arrow_marker.pose = Pose(Point(0.075, 0.0, 0.01), Quaternion(0, 0, 0, 1))
		self.arrow_marker.scale.x = .25
		self.arrow_marker.scale.y = .05
		self.arrow_marker.scale.z = .05
		self.arrow_marker.color.r = 0.0
		self.arrow_marker.color.g = 0.0
		self.arrow_marker.color.b = 1.0
		self.arrow_marker.color.a = 1.0
		self.arrow_marker.lifetime = rospy.Duration(20)

		if self.goal_pose is not None:
			origin_frame = "/map" if mapping else "/odom"
			self.goal_marker = Marker()
			self.goal_marker.header.frame_id = origin_frame
			self.goal_marker.header.stamp = rospy.Time(0)
			self.goal_marker.ns = "goal_circle"
			self.goal_marker.action = Marker.ADD
			self.goal_marker.pose = self.goal_pose
			self.goal_marker.type=3  
			self.goal_marker.scale.x = .15
			self.goal_marker.scale.y = .15
			self.goal_marker.scale.z = .01
			self.goal_marker.color.r = 0.0
			self.goal_marker.color.g = 1.0
			self.goal_marker.color.b = 0.0
			self.goal_marker.color.a = 0.5
			self.goal_marker.lifetime = rospy.Duration(20)

			self.goal_marker_arrow = Marker()
			self.goal_marker_arrow.header.frame_id = origin_frame
			self.goal_marker_arrow.header.stamp = rospy.Time(0)
			self.goal_marker_arrow.ns = "goal_arrow"
			self.goal_marker_arrow.action = Marker.ADD
			self.goal_marker_arrow.pose = self.goal_pose_arrow
			self.goal_marker_arrow.type=0 
			self.goal_marker_arrow.scale.x = .25
			self.goal_marker_arrow.scale.y = .05
			self.goal_marker_arrow.scale.z = .05
			self.goal_marker_arrow.color.r = 0.0
			self.goal_marker_arrow.color.g = 1.0
			self.goal_marker_arrow.color.b = 0.0
			self.goal_marker_arrow.color.a = 0.5
			self.goal_marker_arrow.lifetime = rospy.Duration(20)


			self.goal_circle_pub.publish(self.goal_marker)
			self.goal_arrow_pub.publish(self.goal_marker_arrow)



		self.circle_pub.publish(self.slam_marker)
		self.arrow_pub.publish(self.arrow_marker)

	def goal_callback(self, msg):
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
			self.make_marker()


if __name__ == '__main__':
	marker = Turtlebot_Marker()
	marker.run()