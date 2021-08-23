import unittest
from collections import defaultdict
from typing import Tuple, List

from ictsm.compact_location import CompactLocation
from ictsm.solver import enumerate_matchings, matching_gen


class MatchingTest(unittest.TestCase):
    def test_three_agents_one_team(self):
        goals = [(36875, 0), (73731, 0), (45073, 0)]
        goal_teams = defaultdict(list)
        for goal in goals:
            goal_teams[goal[1]].append(goal[0])
        matchings = list(matching_gen(goal_teams))
        self.assertEqual(len(matchings), 6)

    def test_four_agents_two_teams(self):
        goals = [(36875, 0), (73731, 0), (45073, 1), (36875, 1)]
        goal_teams = defaultdict(list)
        for goal in goals:
            goal_teams[goal[1]].append(goal[0])
        matchings = list(matching_gen(goal_teams))
        self.assertEqual(len(matchings), 4)

    def test_many_matches(self):
        goals = [(i, 0) for i in range(12)]
        goal_teams = defaultdict(list)
        for goal in goals:
            goal_teams[goal[1]].append(goal[0])
        matchings = matching_gen(goal_teams)
        while matchings:
            next(matchings)
