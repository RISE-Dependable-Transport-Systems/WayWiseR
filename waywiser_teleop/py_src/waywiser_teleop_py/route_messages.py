"""Helpers for building WayWise route messages from map waypoints."""

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from waywiser_core.msg import PathWithTwists


def build_path_with_twists(points, stamp, frame_id='map', altitude=0.0, speed=1.0):
    """Create a PathWithTwists message from iterable objects with x/y fields."""
    msg = PathWithTwists()
    msg.path = Path()
    msg.path.header.stamp = stamp
    msg.path.header.frame_id = frame_id

    for point in points:
        pose = PoseStamped()
        pose.header = msg.path.header
        pose.pose.position.x = float(point.x)
        pose.pose.position.y = float(point.y)
        pose.pose.position.z = float(altitude)
        pose.pose.orientation.w = 1.0
        msg.path.poses.append(pose)

        twist = Twist()
        twist.linear.x = float(speed)
        msg.twists.append(twist)

    return msg
