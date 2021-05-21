import unittest

from mapfmclient import MarkedLocation, Problem
from mapfm.solver import solve

"""
Basic single-agent pathfinding tests using top-level 'solve' interface
"""


class SolveTestSingle(unittest.TestCase):
    def setUp(self) -> None:
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
        problem: Problem = self.instance_creator(start, goal)
        solution = solve(problem)
        self.assertListEqual(solution.paths[0].route, [(1, 1), (1, 1)])

    def test_solve_one(self):
        start: MarkedLocation = MarkedLocation(0, 1, 1)
        goal: MarkedLocation = MarkedLocation(0, 1, 2)
        problem: Problem = self.instance_creator(start, goal)
        solution = solve(problem)
        self.assertListEqual(solution.paths[0].route, [(1, 1), (1, 2)])

    def test_solve_many(self):
        start: MarkedLocation = MarkedLocation(0, 1, 1)
        goal: MarkedLocation = MarkedLocation(0, 3, 3)
        problem: Problem = self.instance_creator(start, goal)
        solution = solve(problem)
        self.assertListEqual(
            solution.paths[0].route, [(1, 1), (1, 2), (2, 2), (3, 2), (3, 3)]
        )


"""
Elementary two-agent tests
"""


class SolveTestTwo(unittest.TestCase):
    def setUp(self) -> None:
        def creator(start1, start2, goal1, goal2):
            return Problem(
                [
                    [1, 1, 1, 1],
                    [1, 0, 0, 1],
                    [1, 0, 0, 1],
                    [1, 1, 1, 1],
                ],
                4,
                4,
                [start1, start2],
                [goal1, goal2],
                0,
                2,
                2,
            )

        self.instance_creator = creator

    def test_solve_zero(self):
        start: MarkedLocation = MarkedLocation(0, 1, 1)
        goal: MarkedLocation = MarkedLocation(0, 1, 2)
        problem: Problem = self.instance_creator(start, goal, start, goal)
        solution = solve(problem)
        routes = list(map(lambda pi: pi.route, solution.paths))
        self.assertListEqual(routes, [[(1, 1), (1, 1)], [(1, 2), (1, 2)]])

    def test_solve_one(self):
        problem: Problem = self.instance_creator(
            MarkedLocation(0, 1, 1),
            MarkedLocation(0, 1, 2),
            MarkedLocation(0, 2, 1),
            MarkedLocation(0, 2, 2),
        )
        solution = solve(problem)
        routes = list(map(lambda pi: pi.route, solution.paths))
        self.assertListEqual(routes, [[(1, 1), (2, 1)], [(1, 2), (2, 2)]])

    def test_solve_many(self):
        problem: Problem = self.instance_creator(
            MarkedLocation(0, 1, 1),
            MarkedLocation(1, 1, 2),
            MarkedLocation(0, 2, 2),
            MarkedLocation(1, 2, 1),
        )
        solution = solve(problem)
        routes = list(map(lambda pi: pi.route, solution.paths))
        self.assertListEqual(
            routes, [[(1, 1), (2, 1), (2, 2)], [(1, 2), (1, 1), (2, 1)]]
        )

    def test_solve_swap(self):
        problem: Problem = self.instance_creator(
            MarkedLocation(0, 1, 1),
            MarkedLocation(0, 2, 2),
            MarkedLocation(0, 2, 2),
            MarkedLocation(0, 1, 1),
        )
        solution = solve(problem)
        s = sum(list(map(lambda pi: len(pi.route), solution.paths)))
        self.assertEqual(s, 4)
