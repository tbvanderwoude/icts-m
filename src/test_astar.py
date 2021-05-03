import unittest
from typing import List

from src.astar import astar
from src.compact_location import CompactLocation, compact_location
from src.maze import Maze


class AStarTest(unittest.TestCase):
    def setUp(self) -> None:
        grid: List[List[int]] = [
            [1, 1, 1, 1, 1],
            [1, 0, 1, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 0, 1, 0, 1],
            [1, 1, 1, 1, 1],
        ]
        self.maze: Maze = Maze(grid, 5, 5)

    def test_astar_zero(self):
        start: CompactLocation = compact_location(1, 1)
        goal: CompactLocation = compact_location(1, 1)
        path = astar(self.maze, start, goal)
        self.assertEqual(len(path), 1)

    def test_astar_one(self):
        start: CompactLocation = compact_location(1, 1)
        goal: CompactLocation = compact_location(1, 2)
        path = astar(self.maze, start, goal)
        self.assertEqual(len(path), 2)

    def test_astar_many(self):
        start: CompactLocation = compact_location(1, 1)
        goal: CompactLocation = compact_location(3, 3)
        path = astar(self.maze, start, goal)
        self.assertEqual(len(path), 5)
