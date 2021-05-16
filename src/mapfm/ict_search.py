from collections import deque
from itertools import combinations
from typing import List, Tuple, Optional

from mapfm.compact_location import CompactLocation
from mapfm.id_context import IDContext
from mapfm.maze import Maze
from mapfm.mdd import MDD
from mapfm.mdd_search import (
    JointSolution,
    seek_solution_in_joint_mdd,
    JointTimedSolution,
)


def find_number_of_open_spaces(maze: Maze):
    return sum([sum([x ^ 1 for x in row]) for row in maze.grid])


class ICTSolution:
    def __init__(self, solution: JointSolution, sic: int):
        self.solution = solution
        self.sic = sic


class ICTSearcher(object):
    __slots__ = ["maze", "combs", "prune", "enhanced", "open_spaces", "budget","lower_sic_bound"]

    def __init__(
        self,
        maze: Maze,
        combs: int,
        prune: bool,
        enhanced: bool,
        budget: Optional[int] = None,
    ):
        self.maze = maze
        self.combs = combs
        self.prune = prune
        self.enhanced = enhanced
        self.open_spaces = find_number_of_open_spaces(maze)
        self.lower_sic_bound = 0
        self.budget = budget

    def calculate_upper_bound_cost(self, k: int):
        return (k ** 2) * self.open_spaces

    def check_teams(
        self,
        agents,
        team_agent_indices: List[List[int]],
        mdds: List[MDD],
        accumulator: List = [],
        context: Optional[IDContext] = None,
    ):
        for team in team_agent_indices:
            filtered_agents = list(
                filter(lambda x: x in agents, team_agent_indices[team])
            )

            if filtered_agents and not seek_solution_in_joint_mdd(
                [mdds[i] for i in filtered_agents],
                False,
                self.enhanced,
                accumulator,
                context,
            ):
                return False
        return True

    def check_team_combinations(
        self,
        team_agent_indices: List[List[int]],
        agents,
        mdds: List[MDD],
        accumulator: List = [],
        context: Optional[IDContext] = None,
    ):
        for c in combinations(range(len(team_agent_indices)), self.combs):
            if len(c) <= 1:
                continue
            indices = []
            skip = False
            for team_i in c:
                filtered_agents = list(
                    filter(lambda x: x in agents, team_agent_indices[team_i])
                )
                if not filtered_agents:
                    skip = True
                    break
                indices.extend(filtered_agents)
            if skip:
                continue
            if not seek_solution_in_joint_mdd(
                [mdds[i] for i in indices], False, self.enhanced, accumulator, context
            ):
                return False
        return True

    def check_combinations(
        self,
        mdds: List[MDD],
        k: int,
        accumulator: List = [],
        context: Optional[IDContext] = None,
    ):
        for c in combinations(range(k), self.combs):
            if not seek_solution_in_joint_mdd(
                [mdds[i] for i in c], False, self.enhanced, accumulator, context
            ):
                return False
        return True

    def search_tapf(
        self,
        agents,
        team_agent_indices,
        team_goals,
        root,
        context: Optional[IDContext] = None,
    ) -> Optional[ICTSolution]:
        k = len(agents)
        budget = self.calculate_upper_bound_cost(k)
        frontier = deque()
        frontier.append(root)
        visited = set()
        mdd_cache = dict()
        # print(k, budget, root)
        cost = sum(root)
        while frontier:
            node = frontier.popleft()
            # if sum(node) != cost:
            #     cost = sum(node)
            #     print(cost)
            if (self.lower_sic_bound - k) <= sum(node) <= budget and not node in visited:
                accumulator = []
                visited.add(node)
                mdds = []
                for (i, c) in enumerate(node):
                    node_list = list(node)
                    node_list[i] += 1
                    if not tuple(node_list) in visited:
                        frontier.append(tuple(node_list))
                    if not (i, c) in mdd_cache:
                        if (i, c - 1) in mdd_cache:
                            mdd_cache[(i, c)] = MDD(
                                self.maze,
                                i,
                                agents[i][0],
                                team_goals[agents[i][1]],
                                c,
                                mdd_cache[(i, c - 1)],
                            )
                        else:
                            mdd_cache[(i, c)] = MDD(
                                self.maze, i, agents[i][0], team_goals[agents[i][1]], c
                            )
                    mdds.append(mdd_cache[(i, c)])
                if (
                    not self.prune
                    or k <= self.combs
                    or self.check_combinations(mdds, k, accumulator, context)
                ) and (
                    len(team_agent_indices) == 1
                    or self.check_teams(
                        agents, team_agent_indices, mdds, accumulator, context
                    )
                    and (
                        len(team_agent_indices) <= 2
                        or self.check_team_combinations(
                            team_agent_indices, agents, mdds, accumulator, context
                        )
                    )
                ):
                    solution: JointTimedSolution = seek_solution_in_joint_mdd(
                        mdds, True, False, [], context
                    )
                    if solution:
                        return ICTSolution(
                            list(map(lambda x: x[0], solution)), sum(node)
                        )

                for (i, p, c) in accumulator:
                    mdds[i].mdd[p].add(c)
        return None

    def search(
        self,
        subproblems: List[Tuple[CompactLocation, CompactLocation]],
        root: Tuple[int, ...],
        context: Optional[IDContext] = None,
    ) -> Optional[ICTSolution]:
        k = len(subproblems)
        budget = self.calculate_upper_bound_cost(k)
        frontier = deque()
        frontier.append(root)
        visited = set()
        mdd_cache = dict()
        # print(k, budget, root)
        while frontier:
            node = frontier.popleft()
            if (self.lower_sic_bound-k) <= sum(node) <= budget and not node in visited:
                accumulator = []
                visited.add(node)
                mdds = []
                for (i, c) in enumerate(node):
                    node_list = list(node)
                    node_list[i] += 1
                    if not tuple(node_list) in visited:
                        frontier.append(tuple(node_list))
                    if not (i, c) in mdd_cache:
                        if (i, c - 1) in mdd_cache:
                            mdd_cache[(i, c)] = MDD(
                                self.maze,
                                i,
                                subproblems[i][0],
                                {subproblems[i][1]},
                                c,
                                mdd_cache[(i, c - 1)],
                            )
                        else:
                            mdd_cache[(i, c)] = MDD(
                                self.maze, i, subproblems[i][0], {subproblems[i][1]}, c
                            )
                    mdds.append(mdd_cache[(i, c)])
                if (
                    not self.prune
                    or k <= self.combs
                    or self.check_combinations(mdds, k, accumulator, context)
                ):
                    solution: JointTimedSolution = seek_solution_in_joint_mdd(
                        mdds, True, False, [], context
                    )
                    if solution:
                        return ICTSolution(
                            list(map(lambda x: x[0], solution)), sum(node)
                        )

                for (i, p, c) in accumulator:
                    mdds[i].mdd[p].add(c)
        return None
