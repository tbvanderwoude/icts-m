from copy import copy
from typing import List, Tuple, Optional, Dict

from mapfmclient import Problem, Solution

from mapfm.astar import astar
from mapfm.compact_location import compact_location, expand_location, CompactLocation
from mapfm.conflicts import find_conflict
from mapfm.ict_search import ICTSearcher, ICTSolution
from mapfm.id_context import IDContext
from mapfm.maze import Maze
from mapfm.util import index_path


def enumerate_matchings(agents, tasks):
    if agents:
        (name, type), *tail = agents
        results = []
        for (i, (task_name, task_type)) in enumerate(tasks):
            if type == task_type:
                tasks_cp = copy(tasks)
                tasks_cp.pop(i)
                if tail:
                    results.extend(
                        map(
                            lambda rs: [(name, task_name)] + rs,
                            enumerate_matchings(tail, tasks_cp),
                        )
                    )
                else:
                    results.append([(name, task_name)])
        return results
    else:
        return []


def merge_groups(agent_groups, group_i, group_j):
    new_agent_groups = []
    for agent_group in agent_groups:
        if agent_group == group_j:
            new_agent_groups.append(group_i)
        else:
            new_agent_groups.append(agent_group)
    return new_agent_groups


class Solver:
    __slots__ = [
        "problem",
        "k",
        "max_k_solved",
        "maze",
        "combs",
        "prune",
        "enhanced",
        "id",
        "conflict_avoidance",
        "enumerative",
        "ict_searcher",
    ]

    def __init__(
        self,
        problem: Problem,
        combs: int,
        prune: bool,
        enhanced: bool,
        id: bool,
        conflict_avoidance: bool,
        enumerative: bool,
    ):
        self.problem = problem
        self.k = len(problem.starts)
        self.maze: Maze = Maze(problem.grid, problem.width, problem.height)
        self.max_k_solved = 0
        self.combs = combs
        self.prune = prune
        self.conflict_avoidance = conflict_avoidance
        self.id = id
        self.enumerative = enumerative
        self.ict_searcher = ICTSearcher(self.maze, combs, prune, enhanced,self.k)

    def solve(self) -> Optional[Solution]:
        paths: List[List[Tuple[int, int]]] = []
        agents = list(
            map(
                lambda marked: (compact_location(marked.x, marked.y), marked.color),
                self.problem.starts,
            )
        )
        goals = list(
            map(
                lambda marked: (compact_location(marked.x, marked.y), marked.color),
                self.problem.goals,
            )
        )
        if self.enumerative:
            matchings = enumerate_matchings(agents, goals)
            min_sic = None
            min_sol = None
            for matching in matchings:
                sol = self.solve_matching(matching)
                if sol:
                    sic = sol.sic
                    if not min_sic or min_sic > sic:
                        min_sic = sic
                        min_sol = sol.solution
                        self.update_budget(min_sic)
            # print("Enumerative SIC: " + str(min_sic))
            subsols = list(zip(*min_sol))
            for subsol in subsols:
                paths.append(list(map(lambda loc: expand_location(loc), subsol)))
        else:
            teams = set(map(lambda a: a.color, self.problem.starts))
            team_goals = dict(
                [(team, set([g[0] for g in goals if g[1] == team])) for team in teams]
            )
            team_agent_indices = dict(
                [
                    (team, [i for (i, a) in enumerate(agents) if a[1] == team])
                    for team in teams
                ]
            )
            # print(agents, teams, team_goals, team_agent_indices)
            min_sol = self.solve_tapf_instance(agents, team_agent_indices, team_goals)
            if min_sol:
                subsols = list(zip(*min_sol.solution))
                # print("Native SIC: " + str(min_sol.sic))
                for subsol in subsols:
                    paths.append(list(map(lambda loc: expand_location(loc), subsol)))
            else:
                return None, self.ict_searcher.max_delta,self.max_k_solved
        return Solution.from_paths(paths), self.ict_searcher.max_delta,self.max_k_solved

    def update_budget(self, budget):
        self.ict_searcher.budget = budget

    def update_lower_sic(self, lower_sic):
        self.ict_searcher.lower_sic_bound = lower_sic

    def solve_matching(self, matching: List[Tuple[CompactLocation, CompactLocation]]):
        if self.id:
            return self.solve_mapf_with_id(matching)
        else:
            return self.solve_mapf(matching)

    def solve_tapf_instance(self, agents, team_agent_indices, team_goals):
        if self.id:
            return self.solve_tapf_with_id(agents, team_agent_indices, team_goals)
        else:
            return self.solve_tapf(agents, team_agent_indices, team_goals)

    def solve_tapf(
        self,
        agents,
        team_agent_indices,
        team_goals,
        context: Optional[IDContext] = None,
    ):
        subproblems = []
        root_list = []
        for agent in agents:
            min_len = None
            for goal in team_goals[agent[1]]:
                subproblems.append((agent[0], goal))
                shortest_path = astar(self.maze, agent[0], goal)
                if not shortest_path:
                    return None
                assert len(shortest_path) > 0
                if not min_len or min_len >= len(shortest_path):
                    min_len = len(shortest_path)
            root_list.append(min_len - 1)
        root = tuple(root_list)
        return self.ict_searcher.search_tapf(
            agents, team_agent_indices, team_goals, root, context
        )

    def solve_mapf(
        self,
        matching: List[Tuple[CompactLocation, CompactLocation]],
        context: Optional[IDContext] = None
    ) -> Optional[ICTSolution]:
        subproblems = []
        root_list = []
        for (start, goal) in matching:
            subproblems.append((start, goal))
            shortest = astar(self.maze, start, goal)
            if not shortest:
                return None
            assert len(shortest) > 0
            root_list.append(len(shortest) - 1)
        root = tuple(root_list)
        return self.ict_searcher.search(subproblems, root, context)

    def solve_tapf_group(
        self,
        group: int,
        agent_groups: List[int],
        agents,
        team_agent_indices,
        team_goals,
        context: IDContext,
        lower_sic_bound = 0,
    ):
        # print(agents)
        agent_group = [x for (i, x) in enumerate(agents) if agent_groups[i] == group]
        local_team_agent_indices = dict(
            [
                (team, [i for (i, a) in enumerate(agents) if a[1] == team])
                for team in team_agent_indices.keys()
            ]
        )
        self.update_lower_sic(lower_sic_bound)
        return self.solve_tapf(
            agent_group, local_team_agent_indices, team_goals, context
        )

    def solve_tapf_with_id(
        self,
        all_agents,
        team_agent_indices,
        team_goals,
    ):
        agent_groups = list(range(self.k))
        agent_paths: List[List[Tuple[CompactLocation]]] = []
        group_sic: Dict[int, int] = {}
        for (i, a) in enumerate(all_agents):
            solution = self.solve_tapf([a], team_agent_indices, team_goals)
            if solution:
                agent_paths.append(list(map(lambda x: x[0], solution.solution)))
                group_sic[i] = solution.sic
            else:
                return None
        kprime = 1
        while True:
            unique_groups = set(agent_groups)
            lens = [len(path) for path in agent_paths]
            max_len = max(lens)
            prev = tuple([path[0] for path in agent_paths])
            conflict = False
            conflicting_pair = None
            final_path = [prev]
            for t in range(1, max_len):
                node = tuple(
                    [index_path(agent_paths[i], t, lens[i]) for i in range(self.k)]
                )
                conflicting_pair = find_conflict(node, prev)
                if conflicting_pair:
                    conflict = True
                    break
                else:
                    final_path.append(node)
                    prev = node
            if conflict and len(unique_groups) > 1:
                merged_group = agent_groups[conflicting_pair[0]]
                conflict_group = agent_groups[conflicting_pair[1]]
                # print("{} into {}".format(merged_group,conflict_group))
                lower_sic_bound = group_sic[merged_group] + group_sic[conflict_group]
                group_sic[conflict_group] = 0
                agent_groups = merge_groups(agent_groups, merged_group, conflict_group)
                group_agent_indices = [
                    i
                    for (i, a) in enumerate(all_agents)
                    if agent_groups[i] == merged_group
                ]
                agents = [
                    a
                    for (i, a) in enumerate(all_agents)
                    if agent_groups[i] == merged_group
                ]
                # print(group_agent_indices,agents)
                k_solved = len(agents)
                self.max_k_solved = max(k_solved,self.max_k_solved)
                context = None
                if self.conflict_avoidance and k_solved < self.k:
                    other_agents = [
                        i for i in range(self.k) if agent_groups[i] != merged_group
                    ]
                    context = IDContext(other_agents, agent_paths, lens)
                group_sol = self.solve_tapf_group(
                    agent_groups[conflicting_pair[0]],
                    agent_groups,
                    all_agents,
                    team_agent_indices,
                    team_goals,
                    context,
                    0
                )
                if not group_sol:
                    return None
                group_sic[merged_group] = group_sol.sic
                group_agent_paths = list(zip(*group_sol.solution))

                for (i, agent_index) in enumerate(group_agent_indices):
                    agent_paths[agent_index] = group_agent_paths[i]
                kprime = max(kprime, k_solved)
            else:
                break
        # print("k': {}".format(kprime))
        # print(group_sic)
        return ICTSolution(final_path, sum(group_sic.values()))

    def solve_mapf_group(
        self,
        group: int,
        agent_groups: List[int],
        matching: List[Tuple[CompactLocation, CompactLocation]],
        context: IDContext,
        lower_sic_bound = 0,
    ):
        sub_matching = [x for (i, x) in enumerate(matching) if agent_groups[i] == group]
        self.update_lower_sic(lower_sic_bound)
        return self.solve_mapf(sub_matching, context)

    def solve_mapf_with_id(
        self,
        matching: List[Tuple[CompactLocation, CompactLocation]],
    ):
        agent_groups = list(range(self.k))
        agent_paths: List[List[Tuple[CompactLocation]]] = []
        group_sic: Dict[int, int] = {}
        for (i, match) in enumerate(matching):
            solution = self.solve_mapf([match])
            if solution:
                agent_paths.append(list(map(lambda x: x[0], solution.solution)))
                group_sic[i] = solution.sic
            else:
                return None
        kprime = 1
        while True:
            unique_groups = set(agent_groups)
            lens = [len(path) for path in agent_paths]
            max_len = max(lens)
            prev = tuple([path[0] for path in agent_paths])
            conflict = False
            conflicting_pair = None
            final_path = [prev]
            for t in range(1, max_len):
                node = tuple(
                    [index_path(agent_paths[i], t, lens[i]) for i in range(self.k)]
                )
                conflicting_pair = find_conflict(node, prev)
                if conflicting_pair:
                    conflict = True
                    break
                else:
                    final_path.append(node)
                    prev = node
            if conflict and len(unique_groups) > 1:
                merged_group = agent_groups[conflicting_pair[0]]
                conflict_group = agent_groups[conflicting_pair[1]]
                # print("{} into {}".format(merged_group,conflict_group))
                lower_sic_bound = group_sic[merged_group] + group_sic[conflict_group]
                group_sic[conflict_group] = 0
                agent_groups = merge_groups(agent_groups, merged_group, conflict_group)
                agents = [i for i in range(self.k) if agent_groups[i] == merged_group]

                k_solved = len(agents)
                self.max_k_solved = max(k_solved,self.max_k_solved)
                context = None
                if self.conflict_avoidance and k_solved < self.k:
                    other_agents = [
                        i for i in range(self.k) if agent_groups[i] != merged_group
                    ]
                    context = IDContext(other_agents, agent_paths, lens)
                group_sol = self.solve_mapf_group(
                    agent_groups[conflicting_pair[0]], agent_groups, matching, context,0
                )
                if not group_sol:
                    return None
                group_sic[merged_group] = group_sol.sic
                group_agent_paths = list(zip(*group_sol.solution))

                for (i, agent) in enumerate(agents):
                    agent_paths[agent] = group_agent_paths[i]
                kprime = max(kprime, k_solved)
            else:
                break
        # print("k': {}".format(kprime))
        # print(group_sic)
        return ICTSolution(final_path, sum(group_sic.values()))


def solve_api(problem: Problem) -> Solution:
    solver = Solver(problem, 3, True, True, True, True, True)
    return solver.solve()[0]

def solve(problem: Problem) -> Solution:
    solver = Solver(problem, 3, True, True, True, True, False)
    return solver.solve()

def solve_enum(problem: Problem) -> Solution:
    solver = Solver(problem, 3, True, True, True, True, True)
    return solver.solve()

