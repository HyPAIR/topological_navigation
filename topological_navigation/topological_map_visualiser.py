#!/usr/bin/env python3
""" Node which visualises a topological map in RViz.

Author: Charlie Street
Owner: Charlie Street
"""

import rclpy.time
from topological_navigation.topological_map import TopologicalMap
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from rclpy.node import Node
import rclpy


class TopologicalMapVisualiser(Node):
    """Visualises the nodes, edges and influence areas of a topological map.

    Attributes:
        _top_map: The topological map
        _viz_pub: The publisher for the visualisation
    """

    def __init__(self):
        """Initialise the node."""
        super().__init__("topological_localisation")

        self.declare_parameter("map", rclpy.Parameter.Type.STRING)

        # This allows us to recreate latching from ROS 1
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._top_map = TopologicalMap(self.get_parameter("map").value)

        self._viz_pub = self.create_publisher(
            MarkerArray, "topological_map_markers", qos_profile=latching_qos
        )

        self._publish_map_markers()

        self.get_logger().info("Topological Map Markers Published.")

    def _create_node_marker(self, node_id, marker_id):
        """Create a marker for an individual node.

        Args:
            node_id: The node id in the topological map
            marker_id: The id for the marker in RViz

        Returns:
            The marker for the node
        """
        node_marker = Marker()
        node_marker.header.frame_id = "/map"
        node_marker.header.stamp = self.get_clock().now().to_msg()
        node_marker.type = 2  # Sphere
        node_marker.id = marker_id
        node_marker.action = 0
        node_marker.pose.position.x = self._top_map._nodes[node_id].x
        node_marker.pose.position.y = self._top_map._nodes[node_id].y
        node_marker.pose.position.z = 0.05
        node_marker.pose.orientation.w = 1.0
        node_marker.scale.x = 0.13
        node_marker.scale.y = 0.13
        node_marker.scale.z = 0.13

        # Set the color
        node_marker.color.r = 0.4
        node_marker.color.g = 0.23
        node_marker.color.b = 0.72
        node_marker.color.a = 0.9

        return node_marker

    def _create_edge_marker(self, marker_id):
        """Create a marker which displays the topological edges.

        Args:
            marker_id: The ID for the marker in RViz

        Returns:
            The marker for the edges
        """
        edge_marker = Marker()
        edge_marker.header.frame_id = "/map"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.type = 5  # Line list
        edge_marker.id = marker_id
        edge_marker.action = 0
        edge_marker.scale.x = 0.05
        edge_marker.color.r = 0.96
        edge_marker.color.g = 0.26
        edge_marker.color.b = 0.21
        edge_marker.color.a = 0.3

        for edge in self._top_map._edges:
            src, trg = self._top_map._edges[edge]
            src_point = Point()
            src_point.x = self._top_map._nodes[src].x
            src_point.y = self._top_map._nodes[src].y
            src_point.z = 0.03
            trg_point = Point()
            trg_point.x = self._top_map._nodes[trg].x
            trg_point.y = self._top_map._nodes[trg].y
            trg_point.z = 0.03

            edge_marker.points.append(src_point)
            edge_marker.points.append(trg_point)

        return edge_marker

    def _create_influence_marker(self, node_id, marker_id):
        """Create a marker showing the influence radius for a given node.

        Args:
            Args:
            node_id: The node id in the topological map
            marker_id: The id for the marker in RViz

        Returns:
            The marker for the influence radius
        """
        influence_marker = Marker()
        influence_marker.header.frame_id = "/map"
        influence_marker.header.stamp = self.get_clock().now().to_msg()
        influence_marker.type = 2
        influence_marker.id = marker_id
        influence_marker.action = 0
        influence_marker.pose.position.x = self._top_map._nodes[node_id].x
        influence_marker.pose.position.y = self._top_map._nodes[node_id].y
        influence_marker.pose.position.z = 0.0
        influence_marker.pose.orientation.w = 1.0
        influence_marker.scale.x = self._top_map._influence_radius
        influence_marker.scale.y = self._top_map._influence_radius
        influence_marker.scale.z = 0.02

        # Set the color
        influence_marker.color.r = 1.0
        influence_marker.color.g = 0.44
        influence_marker.color.b = 0.0
        influence_marker.color.a = 0.7

        return influence_marker

    def _publish_map_markers(self):
        """Publish markers for the nodes, edges, and influence areas."""

        marker_array = MarkerArray()
        marker_id = 0

        # Start with the nodes
        for node in self._top_map._nodes:
            marker_array.markers.append(self._create_node_marker(node, marker_id))
            marker_id += 1

        # Add the edges
        marker_array.markers.append(self._create_edge_marker(marker_id))
        marker_id += 1

        # Finally add the influence area
        for node in self._top_map._nodes:
            marker_array.markers.append(self._create_influence_marker(node, marker_id))
            marker_id += 1

        self._viz_pub.publish(marker_array)


def main(args=None):
    """Start up and run the visualiser.

    Args:
        args: Cmd line args
    """
    rclpy.init(args=args)
    visualiser = TopologicalMapVisualiser()
    rclpy.spin(visualiser)

    visualiser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
