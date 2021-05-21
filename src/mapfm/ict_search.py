from collections import deque
from itertools import combinations
from typing import List, Tuple, Optional, Dict, Deque

from mapfm.id_context import IDContext
from mapfm.mapfm_problem import MAPFMProblem
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
    __slots__ = [
        "maze",
        "combs",
        "team_combs",
        "prune",
        "enhanced",
        "open_spaces",
        "budget",
        "max_delta",
        "lower_sic_bound",
        "debug",
        "mdd_cache",
    ]

    def __init__(
        self,
        maze: Maze,
        combs: int,
        prune: bool,
        enhanced: bool,
        max_k: int,
        debug: bool,
        budget: Optional[int] = None,
    ):
        self.maze = maze
        self.max_delta: List[int] = [0] * (max_k + 1)
        self.combs = combs
        self.team_combs = combs
        self.prune = prune
        self.enhanced = enhanced
        self.open_spaces = find_number_of_open_spaces(maze)
        self.lower_sic_bound = 0
        self.debug = debug
        self.budget = budget
        self.mdd_cache: Dict[Tuple[int, int], MDD] = dict()

    def calculate_upper_bound_cost(self, k: int):
        return (k ** 2) * self.open_spaces

    def check_within_teams(
        self,
        problem: MAPFMProblem,
        mdds: List[MDD],
        accumulator: List = [],
        context: Optional[IDContext] = None,
    ):
        for combs in range(2, 4):
            for team in problem.team_agent_indices:
                indices = problem.team_agent_indices[team]
                team_size = len(indices)
                if team_size < combs:
                    continue
                for c in combinations(indices, combs):
                    if not seek_solution_in_joint_mdd(
                        [mdds[i] for i in c], False, self.enhanced, accumulator, context
                    ):
                        return c
        return None

    def check_team_combinations(
        self,
        problem: MAPFMProblem,
        mdds: List[MDD],
        accumulator: List = [],
        context: Optional[IDContext] = None,
    ):
        for combs in range(2, min(4, problem.n_teams)):
            for c in combinations(problem.teams, combs):
                indices = []
                skip = False
                for team_i in c:
                    filtered_agents = list(
                        filter(lambda x: x in problem.agents, problem.team_agent_indices[team_i])
                    )
                    if not filtered_agents:
                        skip = True
                        break
                    indices.extend(filtered_agents)
                if skip:
                    continue
                if not seek_solution_in_joint_mdd(
                    [mdds[i] for i in indices],
                    False,
                    self.enhanced,
                    accumulator,
                    context,
                ):
                    return indices
        return None

    def check_combinations(
        self,
        mdds: List[MDD],
        k: int,
        accumulator: List = [],
        context: Optional[IDContext] = None,
        agents=None,
    ):
        if agents:
            if k > 2:
                for combs in [2, 3]:
                    for c in combinations(range(k), combs):
                        team = agents[c[0]][1]
                        skip = True
                        for i in c[1:]:
                            if agents[i][1] != team:
                                skip = False
                                break

                        if not skip and not seek_solution_in_joint_mdd(
                            [mdds[i] for i in c],
                            False,
                            self.enhanced,
                            accumulator,
                            context,
                        ):
                            return c
        else:
            if k > self.combs:
                for c in combinations(range(k), self.combs):
                    if not seek_solution_in_joint_mdd(
                        [mdds[i] for i in c], False, self.enhanced, accumulator, context
                    ):
                        return c
        return None

    def get_budget(self, k: int):
        if self.budget:
            return self.budget
        else:
            return self.calculate_upper_bound_cost(k)

    def get_node_mdds(self, node, problem):
        mdds = []
        for (i, c) in enumerate(node):
            if not (i, c) in self.mdd_cache:
                if (i, c - 1) in self.mdd_cache:
                    self.mdd_cache[(i, c)] = MDD(
                        self.maze,
                        i,
                        problem.agents[i][0],
                        problem.team_goals[problem.agents[i][1]],
                        c,
                        self.mdd_cache[(i, c - 1)],
                    )
                else:
                    self.mdd_cache[(i, c)] = MDD(
                        self.maze, i, problem.agents[i][0], problem.team_goals[problem.agents[i][1]], c
                    )
            mdds.append(self.mdd_cache[(i, c)])
        return mdds

    def clear_cache(self):
        self.mdd_cache = dict()

    def search_tapf(
        self,
        problem: MAPFMProblem,
        root,
        context: Optional[IDContext] = None
    ) -> Optional[ICTSolution]:
        self.clear_cache()
        k = problem.k
        budget = self.get_budget(k)
        frontier: Deque[Tuple[int, ...]] = deque()
        frontier.append(root)
        visited = set()
        if self.debug:
            print(root)
        root_cost = sum(root)
        cost = root_cost
        while frontier:
            node = frontier.popleft()
            node_sum = sum(node)
            if node_sum <= budget and not node in visited:
                self.max_delta[k] = max(self.max_delta[k], sum(node) - root_cost)
                if self.debug:
                    if node_sum > cost:
                        cost = sum(node)
                        print(cost)
                accumulator = []
                visited.add(node)
                mdds = self.get_node_mdds(node, problem)

                conflict_team_combination = None
                if len(problem.team_agent_indices) > 1:
                    conflict_team_combination = self.check_within_teams(
                        problem,
                        mdds,
                        accumulator,
                        context,
                    )

                if not conflict_team_combination:
                    conflict_combination = None
                    if self.prune and k > self.combs:
                        conflict_combination = self.check_combinations(
                            mdds, k, accumulator, context, problem.agents
                        )

                    if not conflict_combination:
                        conflict_team_comb = None
                        if len(problem.team_agent_indices) > self.team_combs:
                            conflict_team_comb = self.check_team_combinations(
                                problem, mdds, accumulator, context
                            )
                        if not conflict_team_comb:
                            if self.lower_sic_bound <= node_sum:
                                solution: JointTimedSolution = seek_solution_in_joint_mdd(
                                    mdds, True, False, [], context
                                )
                                if solution:
                                    return ICTSolution(
                                        list(map(lambda x: x[0], solution)), sum(node)
                                    )
                            for (i, c) in enumerate(node):
                                node_list = list(node)
                                node_list[i] += 1
                                if not tuple(node_list) in visited:
                                    frontier.append(tuple(node_list))
                        else:
                            for team in conflict_team_comb:
                                for i in problem.team_agent_indices[team]:
                                    node_list = list(node)
                                    node_list[i] += 1
                                    if not tuple(node_list) in visited:
                                        frontier.append(tuple(node_list))
                    else:
                        for i in conflict_combination:
                            node_list = list(node)
                            node_list[i] += 1
                            if not tuple(node_list) in visited:
                                frontier.append(tuple(node_list))
                else:
                    for i in conflict_team_combination:
                        node_list = list(node)
                        node_list[i] += 1
                        if not tuple(node_list) in visited:
                            frontier.append(tuple(node_list))
                for (i, p, c) in accumulator:
                    mdds[i].mdd[p].add(c)
        return None
