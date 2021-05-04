import unittest
from typing import List, Tuple

from mapfm.compact_location import CompactLocation, compact_location
from mapfm.conflicts import *


def construct_node(ns: List[Tuple[int, int]]) -> List[CompactLocation]:
    return [compact_location(x[0], x[1]) for x in ns]


class ConflictTest(unittest.TestCase):
    def test_all_different(self):
        xs = construct_node([(0, 0), (0, 0), (1, 1)])
        self.assertFalse(all_different(xs))
        self.assertEqual(all_different_constructive(xs),(0,1))

    def test_edge_conflict(self):
        xs = construct_node([(0, 0), (0, 1), (2, 1)])
        ys = construct_node([(0, 1), (2, 1), (0, 1)])
        self.assertTrue(has_edge_collisions(xs,ys))
        self.assertEqual(edge_collisions_constructive(xs,ys),(1,2))

    def test_find_conflict_edge(self):
        xs = construct_node([(0, 0), (0, 1), (2, 1)])
        ys = construct_node([(0, 1), (2, 1), (0, 1)])
        self.assertEqual(find_conflict(xs, ys), (1, 2))

    def test_find_conflict_nod(self):
        xs = construct_node([(0, 0), (0, 0), (1, 1)])
        ys = construct_node([(2,3),(3,4),(4,5)])
        self.assertEqual(find_conflict(xs, ys), (0,1))
