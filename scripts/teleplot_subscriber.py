#!/usr/bin/env python
from __future__ import print_function, division

import socket
from enum import Enum

import rospy
from geometry_msgs.msg import Twist
from husky_msgs.msg import HuskyStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, NavSatFix
from rosgraph_msgs.msg import Clock

# Teleplot socket parameters
TELEPLOT_ADDR = ("127.0.0.1", 47269)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class PlotMode(Enum):
    TIME = "g"
    XY = "xy"


def send_telemetry(name, time, value, mode=PlotMode.TIME):
    """Send telemetry data as a UDP packet

    Args:
        name (str): Variable name
        time (int): time, in milliseconds
        value (float): value to plot
        mode (PlotMode enum value, optional): Plotting mode. Defaults to PlotMode.TIME.
    """
    pkt_message = "{name}:{time}:{value}|{mode}".format(
        name=name, time=time, value=value, mode=mode.value
    )
    sock.sendto(pkt_message.encode(), TELEPLOT_ADDR)


def rostime_to_msecs(rostime):
    """Convert rostime to milliseconds

    Args:
        rostime (rospy.Time): Rospy Time object to convert

    Returns:
        int: time in milliseconds
    """
    return rostime.to_nsec() // 1e6


class HuskySuscriber:
    """Process data from a rosbag and send it to Teleplot

    This example was developed with the rosbags from the Enav Planetary Dataset
    """

    def cmd_vel_cback(self, msg):
        """Commanded velocities of the Husky rover base"""
        time = rostime_to_msecs(rospy.Time.now())
        linear = msg.linear.x
        angular = msg.angular.z
        send_telemetry("lin_cmd", time, linear)
        send_telemetry("ang_cmd", time, angular)


def teleplot_subscriber():
    rospy.init_node("py_subscriber", anonymous=False)

    husky_suscriber = HuskySuscriber()

    rospy.Subscriber("husky_commanded_velocity", Twist, husky_suscriber.cmd_vel_cback)

    rospy.spin()


if __name__ == "__main__":
    teleplot_subscriber()
