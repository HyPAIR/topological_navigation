#!/usr/bin/env python3
""" A node which publishes a robot's current topological location.

Author: Charlie Street
Owner: Charlie Street
"""

from topological_navigation.topological_map import TopologicalMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from rclpy.node import Node
from shapely import Point
import rclpy


class TopologicalLocalisation(Node):
    """A node which publishes the robot's topological location based on its pose.

    Attributes:
        _top_map: The topological map
        _loc_pub: The topological location publisher
        _pose_sub: The robot pose subscriber
        _current_loc: The robot's current location
    """

    def __init__(self):
        """Initialise the node."""
        super().__init__("topological_localisation")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map", rclpy.Parameter.Type.STRING),
                ("pose_topic", "amcl_pose"),
            ],
        )

        # This allows us to recreate latching from ROS 1
        # Both publisher and subscriber need this QoS policy
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._top_map = TopologicalMap(self.get_parameter("map").value)
        self._loc_pub = self.create_publisher(
            String, "topological_location", qos_profile=latching_qos
        )
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self._pose_to_loc,
            latching_qos,
        )
        self._current_loc = None

        self.get_logger().info("Topological Localisation Started.")

    def _pose_to_loc(self, msg):
        """Convert the robot pose to a topological location and publish.

        Args:
            msg: The PoseWithCovarianceStamped message
        """
        point = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
        top_loc = self._top_map.get_location(point)

        # Log changes in location
        if top_loc != self._current_loc:
            self.get_logger().info("New topological location: {}".format(top_loc))
            self._current_loc = top_loc

        loc_msg = String()
        loc_msg.data = str(top_loc)

        self._loc_pub.publish(loc_msg)


def main(args=None):
    """Start up and run localiser.

    Args:
        args: Cmd line args
    """
    rclpy.init(args=args)
    localiser = TopologicalLocalisation()
    rclpy.spin(localiser)

    localiser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
