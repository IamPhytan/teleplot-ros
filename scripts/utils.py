import socket
from enum import Enum

from tf.transformations import euler_from_quaternion

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


def rpy_from_quaternion(quaternion):
    """Get rpy from quaternion msg structure

    Args:
        quaternion (msg structure): ROS msg object that contains x, y, z and w attributes

    Returns:
        (rpy): roll pitch yaw values in a tuple
    """
    q = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    euler = euler_from_quaternion(q)
    roll, pitch, yaw = euler
    return roll, pitch, yaw
