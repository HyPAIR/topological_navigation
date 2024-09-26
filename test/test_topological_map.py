#!/usr/bin/env python3
""" Unit tests for topological_map.py.

Author: Charlie Street
Owner: Charlie Street
"""

from topological_navigation.topological_map import TopologicalMap
from shapely import Point


def test_top_map():
    example_map = "example_map.yaml"

    top_map = TopologicalMap(example_map)

    assert top_map._nodes == {
        "v1": Point(0.0, 0.0),
        "v2": Point(1.0, 1.0),
        "v3": Point(2.0, 2.0),
    }

    assert top_map._edges == {
        "e12": ["v1", "v2"],
        "e23": ["v2", "v3"],
        "e31": ["v3", "v1"],
    }

    assert top_map._influence_radius == 0.3

    assert top_map._graph == {
        "v1": {"e12": "v2", "e31": "v3"},
        "v2": {"e12": "v1", "e23": "v3"},
        "v3": {"e23": "v2", "e31": "v1"},
    }

    assert top_map.at_node(Point(0.2, 0.2), "v1")
    assert not top_map.at_node(Point(0.2, 0.2), "v2")
    assert not top_map.at_node(Point(0.2, 0.2), "v3")

    assert not top_map.at_node(Point(0.8, 0.8), "v1")
    assert top_map.at_node(Point(0.8, 0.8), "v2")
    assert not top_map.at_node(Point(0.8, 0.8), "v3")

    assert not top_map.at_node(Point(1.8, 1.8), "v1")
    assert not top_map.at_node(Point(1.8, 1.8), "v2")
    assert top_map.at_node(Point(1.8, 1.8), "v3")

    assert top_map.get_location(Point(0.2, 0.2)) == "v1"
    assert top_map.get_location(Point(0.8, 0.8)) == "v2"
    assert top_map.get_location(Point(1.8, 1.8)) == "v3"
    assert top_map.get_location(Point(5.0, 5.0)) == None

    assert set(top_map.edges_from_node("v1")) == set(["e12", "e31"])
    assert set(top_map.edges_from_node("v2")) == set(["e12", "e23"])
    assert set(top_map.edges_from_node("v3")) == set(["e23", "e31"])

    assert top_map.get_target("v1", "e12") == "v2"
    assert top_map.get_target("v1", "e31") == "v3"
    assert top_map.get_target("v2", "e12") == "v1"
    assert top_map.get_target("v2", "e23") == "v3"
    assert top_map.get_target("v3", "e23") == "v2"
    assert top_map.get_target("v3", "e31") == "v1"
