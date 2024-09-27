#!/usr/bin/env python3
""" Action server for topological edge navigation.

Author: Charlie Street
Owner: Charlie Street
"""

from topological_navigation.topological_map import TopologicalMap
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.action import ActionServer, ActionClient
from topological_msgs.action import NavigateEdge
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from rclpy.node import Node
from threading import Lock
import rclpy


class EdgeNavigationServer(Node):
    """An action server which handles topological edge action.

    Navigation commands will be terminated once the robot reaches the influence zone.

    Attributes:
        _action_server: The action server
        _top_map: The topological map
        _top_loc_sub: A subscriber to the robot's topological location
        _current_loc: The robot's current topological location
        _current_nav_handle: The current goal sent to the navigation client
        _current_dest: The current destination
        _lock: Lock for testing or writing nav_handle (not wait actions) or current_dest
        _nav_client: An action client for navigation
    """

    def __init__(self):
        """Initialise the action server."""
        super().__init__("edge_navigation_server")
        self.declare_parameter("map", rclpy.Parameter.Type.STRING)

        self._top_map = TopologicalMap(self.get_parameter("map").value)
        self._current_loc = None
        self._current_nav_handle = None
        self._current_dest = None
        self._lock = Lock()

        # This allows us to recreate latching from ROS 1
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._top_loc_sub = self.create_subscription(
            String,
            "topological_location",
            self._receive_top_loc,
            latching_qos,
        )

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_client.wait_for_server()

        self._action_server = ActionServer(
            self,
            NavigateEdge,
            "edge_navigation",
            execute_callback=self._execute_callback,
        )

        self.get_logger().info("Edge Navigation Server Started")

    def destroy(self):
        """Destroy action server."""
        self._action_server.destroy()
        self._nav_client.destroy()
        super().destroy_node()

    def _receive_top_loc(self, msg):
        """Update the topological location and cancel any goals if dest reached.

        Args:
            msg: The topological location
        """
        self._current_loc = msg.data

        with self._lock:
            if self._current_nav_handle is not None:
                if self._current_loc == self._current_dest:
                    self.get_logger().info(
                        "Preempting Navigation: Influence Zone Reached"
                    )
                    # I don't mind whether this goes through or not
                    # If it works, the robot stops early
                    # If it doesn't, the robot will stop once nav completes naturally
                    self._nav_client._cancel_goal_async(self._current_nav_handle)

    def _create_result(self, loc, success):
        """Create a result message.

        Args:
            loc: The destination
            success: Was the action successful

        Returns:
            The result message
        """
        result = NavigateEdge.Result()
        result.dest = loc
        result.success = success
        return result

    def _create_nav_goal(self, dest_node):
        """Create a navigation goal to a given destination node.

        Args:
            dest_node: The node to navigate to

        Returns:
            The navigation goal
        """
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        point = self._top_map._nodes[dest_node]
        nav_goal.pose.pose.position.x = point.x
        nav_goal.pose.pose.position.y = point.y
        return nav_goal

    def _execute_callback(self, goal_handle):
        """Send a navigation goal, and wait until completion or the node is reached.

        Args:
            goal_handle: The handle for the action goal

        Returns:
            The action result
        """
        edge = goal_handle.request.edge_id
        self.get_logger().info("Received request to navigate on {}".format(edge))

        # Check edge is valid. If not, fail immediately
        if (
            self._current_loc not in self._top_map._nodes
            or edge not in self._top_map.edges_from_node(self._current_loc)
        ):
            self.get_logger().info("Action failed due to invalid input")
            goal_handle.abort()
            return self._create_result(self._current_loc, False)

        # Get destination node
        dest_node = self._top_map.get_target(self._current_loc, edge)
        with self._lock:
            self._current_dest = dest_node

        # Send goal and test that it's accepted
        future = self._nav_client.send_goal_async(self._create_nav_goal(dest_node))
        self.executor.spin_until_future_complete(future)
        self._current_nav_handle = future.result()
        if not self._current_nav_handle.accepted:
            self.get_logger().info("Goal Not Accepted")
            goal_handle.abort()
            with self._lock:
                self._current_nav_handle = None
            return self._create_result(self._current_loc, False)

        # Wait for the result, i.e. until navigation is finished
        result = self._current_nav_handle.get_result_async()
        self.executor.spin_until_future_complete(result)
        with self._lock:
            self._current_nav_handle = None
        self.get_logger().info("Navigation Finished")

        # Process result and return
        if (
            self._current_loc == dest_node
            or result.result().status == GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().info("Edge Navigation Successful")
            goal_handle.succeed()
            return self._create_result(dest_node, True)
        else:
            self.get_logger().info("Edge Navigation Failed")
            goal_handle.abort()
            return self._create_result(self._current_loc, False)


def main(args=None):
    rclpy.init(args=args)

    action_server = EdgeNavigationServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()
