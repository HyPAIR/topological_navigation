#!/usr/bin/env python3
"""A fairly bare-bones class for topological maps.

Author: Charlie Street
Owner: Charlie Street
"""

from shapely import Point, distance
import yaml


class TopologicalMap(object):
    """A class for topological maps.

    Topological maps have a set of nodes and edges.
    Robots navigate between nodes using edges.

    All edges in the map are assumed bidirectional

    Attributes:
        _nodes: A dictionary from node name to Points
        _edges: A dictionary from edge name to a (src,trg) pair
        _graph: A dictionary from node to edges from that node to target nodes
        _influence_radius: The radius around a node where we consider the robot at that node
    """

    def __init__(self, map_yaml):
        """Initialise the map.

        Args:
            map_yaml: The map yaml file
        """

        with open(map_yaml, "r") as yaml_in:
            map_dict = yaml.load(yaml_in, Loader=yaml.FullLoader)

        self._nodes = {}
        for node in map_dict["nodes"]:
            xy_dict = map_dict["nodes"][node]
            self._nodes[node] = Point(xy_dict["x"], xy_dict["y"])

        self._edges = map_dict["edges"]
        self._influence_radius = map_dict["influence_radius"]
        self._graph = {}

        for edge in self._edges:
            src, trg = self._edges[edge]

            if src not in self._graph:
                self._graph[src] = {}

            if trg not in self._graph:
                self._graph[trg] = {}

            # Edges are bidirectional
            self._graph[src][edge] = trg
            self._graph[trg][edge] = src

    def at_node(self, point, node):
        """Check if a point is within the influence radius of a node.

        Args:
            point: The point to check
            node: The node id

        Returns:
            True if point is within the influence radius of node
        """
        return distance(point, self._nodes[node]) < self._influence_radius

    def get_location(self, point):
        """Get the current topological node at point.

        We assume the influence radius of nodes do not intersect.
        If the robot is not at a node, we return None.

        Args:
            point: The robot's location

        Returns:
            The robot's topological location
        """

        for node in self._nodes:
            if self.at_node(point, node):
                return node

        return None

    def edges_from_node(self, node):
        """Get the outgoing edges from a node.

        Args:
            node: A node name

        Returns:
            The outgoing edges from node
        """
        return list(self._graph[node].keys())

    def get_target(self, node, edge):
        """Get the target node from navigating along an edge from a node.

        Args:
            node: The start node
            edge: The edge

        Returns:
            target: The intended navigation target
        """
        return self._graph[node][edge]
