#!/usr/bin/env python3
""" Action server for topological edge navigation.

Author: Charlie Street
Owner: Charlie Street
"""

import rclpy.wait_for_message
from topological_navigation.topological_map import TopologicalMap
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.action import ActionServer, ActionClient
from topological_msgs.action import NavigateEdge
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from rclpy.node import Node
from threading import Lock
import rclpy
import time


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

        # I hate this so much
        self._wait_for_nav2_and_location()

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

    def _wait_for_nav2_and_location(self):
        """Wait for nav2 to finish and a topological location to be received."""
        while self._current_loc is None:
            self.get_logger().info("Waiting for Topological Location...")
            rclpy.spin_once(self)
        self.get_logger().info("Topological Location Received")

        self.get_logger().info("Waiting for BT Navigator and AMCL to Setup")
        self._wait_for_node_to_activate("amcl")
        self._wait_for_node_to_activate("bt_navigator")
        self.get_logger().info("BT Navigator and AMCL Ready")

    def _wait_for_node_to_activate(self, node_name):
        """Wait for a particular nav2 node to become active.

        This is horrible but necessary, as the NavigateToPose server will say it
        is ready when its not. I need to wait for other nav2 nodes to be active first.

        This function is taken from:
        https://robotics.snowcron.com/robotics_ros2/waypoint_follower_scripts.htm

        Args:
            node_name: The node to wait for
        """
        self.get_logger().debug("Waiting for {} to become active.".format(node_name))
        node_service = node_name + "/get_state"
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("{} unavailable, waiting.".format(node_service))

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            self.get_logger().debug("Getting {} state...".format(node_name))
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().debug("Result of get_state: {}".format(state))
            time.sleep(2)

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
