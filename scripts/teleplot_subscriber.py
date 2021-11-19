#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from geometry_msgs.msg import Twist
from husky_msgs.msg import HuskyStatus
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, NavSatFix

import utils as u
from utils import PlotMode


class HuskySubscriber:
    """Process data from a rosbag and send it to Teleplot

    This example was developed with the rosbags from the Enav Planetary Dataset
    """

    def cmd_vel_cback(self, msg):
        """Commanded velocities of the Husky rover base"""
        time = u.rostime_to_msecs(rospy.Time.now())
        linear = msg.linear.x
        angular = msg.angular.z
        u.send_telemetry("lin_cmd", time, linear)
        u.send_telemetry("ang_cmd", time, angular)

    def gps_cback(self, msg):
        """GPS data"""
        time = u.rostime_to_msecs(msg.header.stamp)
        lat = msg.latitude
        long = msg.longitude
        alt = msg.altitude
        u.send_telemetry("gps", lat, long, mode=PlotMode.XY)
        u.send_telemetry("altitude", time, alt)

    def odom_cback(self, msg):
        """Odometry data computed from Husky wheel encoders"""
        time = u.rostime_to_msecs(msg.header.stamp)
        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z
        u.send_telemetry("lin_odom", time, linear)
        u.send_telemetry("ang_odom", time, angular)

    def pose_cback(self, msg):
        """Pose estimates from VINS-Fusion"""
        time = u.rostime_to_msecs(msg.header.stamp)
        pose = msg.pose.pose
        pose_x = pose.position.x
        pose_y = pose.position.y
        pose_z = pose.position.z

        orient = msg.pose.pose.orientation
        r, p, y = u.rpy_from_quaternion(orient)

        u.send_telemetry("roll", time, r)
        u.send_telemetry("pitch", time, p)
        u.send_telemetry("yaw", time, y)


def teleplot_subscriber():
    rospy.init_node("py_subscriber", anonymous=False)

    husky_subs = HuskySubscriber()

    rospy.Subscriber("husky_commanded_velocity", Twist, husky_subs.cmd_vel_cback)
    rospy.Subscriber("husky_velocity_estimate", Odometry, husky_subs.odom_cback)
    rospy.Subscriber("gps", NavSatFix, husky_subs.gps_cback)
    rospy.Subscriber("global_odometry_utm", Odometry, husky_subs.pose_cback)

    rospy.spin()


if __name__ == "__main__":
    teleplot_subscriber()
