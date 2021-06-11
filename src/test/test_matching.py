import unittest
from typing import Tuple, List

from ictsm.compact_location import CompactLocation
from ictsm.solver import enumerate_matchings


class MatchingTest(unittest.TestCase):
    def test_three_agents_one_team(self):
        agents, goals = [(24580, 0), (77828, 0), (36880, 0)], [(36875, 0), (73731, 0), (45073, 0)]
        matchings: List[List[Tuple[CompactLocation, CompactLocation]]] = enumerate_matchings(agents, goals)
        self.assertEqual(len(matchings),6)
    def test_four_agents_two_teams(self):
        agents, goals = [(24580, 0), (77828, 0), (36880, 1),(36880, 1)], [(36875, 0), (73731, 0), (45073, 1),(36875, 1)]
        matchings: List[List[Tuple[CompactLocation, CompactLocation]]] = enumerate_matchings(agents, goals)
        self.assertEqual(len(matchings),4)
