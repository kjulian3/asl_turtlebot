#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, Point, Pose2D
from visualization_msgs.msg import Marker

# look up doc.
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt


'''
This code tracks objects in front of the robot and commands the robot to follow the object.
It looks straight ahead within a 30 degree view, and only RHO meters in front.

It broadcasts a TF frame called object, and draws a circle about the mean of the object position.

Although this "works", it is not perfect and it can be significantly improved.
- Currently, it is very noisy, as it considers other points from far away places.
- Currently, the object size is fixed.
- What happens when there are multiple nearby objects?
- Does not visualize the "viewing sector" of the robot.
'''

# min number of points to be considered a object 
MIN_POINTS = 3

# look ahead distance to search for objects
RHO = 1.2

def compute_ellipse_points(a, b):
    th = np.arange(0, 2*np.pi+1, 0.2)
    x = a * np.cos(th)
    y = b * np.sin(th)
    return np.stack([x,y])

def initialize_object_marker():
    object_marker = Marker()
    object_marker.header.frame_id = "/object"
    object_marker.ns = "ellipse"
    object_marker.type = Marker.LINE_STRIP
    object_marker.scale.x = 0.01
    object_marker.frame_locked = True
    object_marker.color.g = 1
    object_marker.color.a = 1
    return object_marker

class TrackOjbect:
    def __init__(self):
        rospy.init_node("track_ojbect")
        self.object_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.object_mean = None
        self.track_ojbect_pub = rospy.Publisher("/viz/object", Marker, queue_size=10)
        self.object_marker = initialize_object_marker()
        rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)


    def velodyne_callback(self, msg):
        '''
        This is an example of how to process PointCloud2 data.
        pc2.read_points creates an _iterator_ object.
        '''
        lidar_info = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        num_points = len(list(lidar_info))
        lidar_info = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        x_coords = np.zeros(num_points)
        y_coords = np.zeros(num_points)
        z_coords = np.zeros(num_points)
        i = 0
        # looping through the point cloud
        for p in lidar_info:

            x_coords[i] = p[0]
            y_coords[i] = p[1]
            z_coords[i] = p[2]
            i += 1

        pt_ranges = np.hypot(x_coords, y_coords)
        pt_angles = np.arctan2(y_coords, x_coords)

        # filter based on range
        pts_within_range = (pt_ranges < RHO)

        # filter based on angle
        pts_within_angle = (pt_angles < np.pi/12) & (pt_angles > -np.pi/12)

        # filter points based on z
        pts_above_ground = (z_coords > -0.05)

        # filtered points
        filtered_points = pts_within_range & pts_within_angle & pts_above_ground

        x_filtered = x_coords[filtered_points]
        y_filtered = y_coords[filtered_points]
        z_filtered = z_coords[filtered_points]
        self.object_time = msg.header.stamp

        if sum(filtered_points) > MIN_POINTS:
            self.object_mean = (np.mean(x_filtered), np.mean(y_filtered), np.mean(z_filtered))
            self.object_var = (np.var(x_filtered), np.var(y_filtered), np.var(z_filtered))



    def loop(self):
        print("MADE It HERE")
        print(self.object_mean)
        if self.object_mean is not None:
            pt = PointStamped()
            pt.header.frame_id = '/velodyne'
            pt.header.stamp = self.object_time
            pt.point.x = self.object_mean[0]
            pt.point.y = self.object_mean[1]
            pt.point.z = self.object_mean[2]




            try:
                # send a tf transform of the object location in the map frame
                self.tf_listener.waitForTransform("/map", '/velodyne', self.object_time, rospy.Duration(.05))
                object_map_pt = self.tf_listener.transformPoint("/map", pt)
                self.object_broadcaster.sendTransform((object_map_pt.point.x, object_map_pt.point.y, object_map_pt.point.z), 
                                                       [0, 0, 0, 1],
                                                       self.object_time,
                                                       "/object",
                                                       "/map")

                # Get rotation of velodyne frame from map frame
                (translation,rotation) = self.tf_listener.lookupTransform("/map", '/velodyne', rospy.Time(0))
                theta = tf.transformations.euler_from_quaternion(rotation)[2]

                # Create the Pose2D, defined in the map frame
                pose_g_msg = Pose2D()
                pose_g_msg.x = object_map_pt.point.x
                pose_g_msg.y = object_map_pt.point.y
                pose_g_msg.theta = np.arctan2(pt.point.y,pt.point.x) + theta

                # Publish the Pose2D as a goal for the controller
                self.pose_goal_publisher.publish(pose_g_msg)
                
                # make object marker
                ellipse_points = compute_ellipse_points(0.2, 0.2)
                self.object_marker.points = []
                for i in range(ellipse_points.shape[-1]):
                    # print("drawing ellipse")
                    self.object_marker.points.append(Point(ellipse_points[0,i], ellipse_points[1,i], 0)) 
                self.track_ojbect_pub.publish(self.object_marker)

            except:
                print("Uh OH")


    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    track_ojbect = TrackOjbect()
    track_ojbect.run()