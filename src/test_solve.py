import unittest
from typing import List

from mapfmclient import MarkedLocation, Problem
from src.solve import solve


class SolveTestSingle(unittest.TestCase):
    def setUp(self) -> None:
        self.grid: List[List[int]] = [
            [1, 1, 1, 1, 1],
            [1, 0, 1, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 0, 1, 0, 1],
            [1, 1, 1, 1, 1],
        ]
        def creator(start, goal):
            return Problem(
            [
                [1, 1, 1, 1, 1],
                [1, 0, 1, 0, 1],
                [1, 0, 0, 0, 1],
                [1, 0, 1, 0, 1],
                [1, 1, 1, 1, 1],
            ],
            5,
            5,
            [start],
            [goal],
            0,
            1,
            1,
        )
        self.instance_creator = creator

    def test_solve_zero(self):
        start: MarkedLocation = MarkedLocation(0, 1, 1)
        goal: MarkedLocation = MarkedLocation(0, 1, 1)
        problem: Problem = self.instance_creator(start,goal)
        solution = solve(problem)
        self.assertListEqual(solution.paths[0].route, [(1,1),(1,1)])

    def test_solve_one(self):
        start: MarkedLocation = MarkedLocation(0, 1, 1)
        goal: MarkedLocation = MarkedLocation(0, 1, 2)
        problem: Problem = self.instance_creator(start,goal)
        solution = solve(problem)
        self.assertListEqual(solution.paths[0].route, [(1,1),(1,2)])

    def test_solve_many(self):
        start: MarkedLocation = MarkedLocation(0, 1, 1)
        goal: MarkedLocation = MarkedLocation(0, 3, 3)
        problem: Problem = self.instance_creator(start,goal)
        solution = solve(problem)
        self.assertListEqual(solution.paths[0].route, [(1, 1), (1, 2),(2,2),(3,2),(3,3)])