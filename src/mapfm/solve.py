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


def solve_api(problem: Problem) -> Solution:
    return solve(problem)[0]


def solve_api_enum(problem: Problem) -> Solution:
    return solve_enum_sorted(problem)[0]


def solve(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=True,
        id=True,
        conflict_avoidance=True,
        enumerative=False,
    )
    return Solver(config, problem)()


def solve_enum_sorted(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=True,
    )
    return Solver(config, problem)()


def solve_enum(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=False,
    )
    return Solver(config, problem)()


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


class SolverConfig:
    __slots__ = [
        "combs",
        "prune",
        "enhanced",
        "id",
        "conflict_avoidance",
        "enumerative",
        "sort_matchings",
    ]

    def __init__(
        self,
        combs: int,
        prune: bool,
        enhanced: bool,
        id: bool,
        conflict_avoidance: bool,
        enumerative: bool,
        sort_matchings: bool = True,
    ):
        self.combs = combs
        self.prune = prune
        self.enhanced = enhanced
        self.id = id
        self.conflict_avoidance = conflict_avoidance
        self.enumerative = enumerative
        self.sort_matchings = sort_matchings


class Solver:
    __slots__ = [
        "config",
        "problem",
        "k",
        "max_k_solved",
        "maze",
        "ict_searcher",
        "path_cache",
    ]

    def __init__(self, config: SolverConfig, problem: Problem):
        self.config: SolverConfig = config
        self.max_k_solved = 0
        self.path_cache = dict()
        self.problem = problem
        self.k = len(problem.starts)
        self.maze: Maze = Maze(problem.grid, problem.width, problem.height)
        self.ict_searcher = ICTSearcher(
            self.maze,
            self.config.combs,
            self.config.prune,
            self.config.enhanced,
            self.k,
        )

    def __call__(self) -> Tuple[Optional[Solution], List[int], int]:
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
        if self.config.enumerative:
            matchings = enumerate_matchings(agents, goals)
            if self.config.sort_matchings:
                rooted_matchings = list(
                    map(lambda m: (m, sum(self.compute_root(m))), matchings)
                )
                rooted_matchings.sort(key=lambda a: a[1])
            else:
                rooted_matchings = list(map(lambda m: (m, 0), matchings))
            min_sic = None
            min_sol = None
            for (matching, _) in rooted_matchings:
                team_agent_indices = dict(map(lambda x: (x,{x}),agents))
                team_goals = dict(map(lambda x: (x[0],{x[1][1]}),enumerate(matching)))
                sol = self.solve_tapf_instance(agents, team_agent_indices, team_goals)
                if sol:
                    sic = sol.sic
                    if not min_sic or min_sic > sic:
                        min_sic = sic
                        min_sol = sol.solution
                        self.update_budget(min_sic)
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
                    (team, set([i for (i, a) in enumerate(agents) if a[1] == team]))
                    for team in teams
                ]
            )
            min_sol = self.solve_tapf_instance(agents, team_agent_indices, team_goals)
            if min_sol:
                subsols = list(zip(*min_sol.solution))
                for subsol in subsols:
                    paths.append(list(map(lambda loc: expand_location(loc), subsol)))
            else:
                return None, self.ict_searcher.max_delta, self.max_k_solved
        return (
            Solution.from_paths(paths),
            self.ict_searcher.max_delta,
            self.max_k_solved,
        )

    def update_budget(self, budget):
        self.ict_searcher.budget = budget

    def update_lower_sic(self, lower_sic):
        self.ict_searcher.lower_sic_bound = lower_sic

    def solve_matching(self, matching: List[Tuple[CompactLocation, CompactLocation]]):
        if self.config.id:
            return self.solve_mapf_with_id(matching)
        else:
            return self.solve_mapf(matching)

    def solve_tapf_instance(self, agents, team_agent_indices, team_goals):
        if self.config.id:
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
        root = self.compute_root_m(agents,team_goals)
        return self.ict_searcher.search_tapf(
            agents, team_agent_indices, team_goals, root, context
        )

    def compute_root_m(self,agents,team_goals):
        root_list = []
        for agent in agents:
            min_c = None
            start = agent[0]
            for goal in team_goals[agent[1]]:
                if not (start, goal) in self.path_cache:
                    shortest_path = astar(self.maze, start, goal)
                    if not shortest_path:
                        return None
                    self.path_cache[(start, goal)] = len(shortest_path)
                c = self.path_cache[(start, goal)]
                assert c > 0
                if not min_c or min_c >= c:
                    min_c = c
            root_list.append(min_c - 1)
        return tuple(root_list)

    def compute_root(
        self,
        matching: List[Tuple[CompactLocation, CompactLocation]],
    ):
        root_list = []
        for (start, goal) in matching:
            if (start, goal) in self.path_cache:
                root_list.append(self.path_cache[(start, goal)])
            else:
                shortest = astar(self.maze, start, goal)
                if not shortest:
                    return None
                assert len(shortest) > 0
                c = len(shortest) - 1
                self.path_cache[(start, goal)] = c
                root_list.append(c)
        return tuple(root_list)

    def solve_mapf(
        self,
        matching: List[Tuple[CompactLocation, CompactLocation]],
        context: Optional[IDContext] = None,
    ) -> Optional[ICTSolution]:
        root = self.compute_root(matching)
        return self.ict_searcher.search(matching, root, context)

    def solve_tapf_group(
        self,
        group: int,
        agent_groups: List[int],
        agents,
        team_agent_indices,
        team_goals,
        context: IDContext,
        lower_sic_bound=0,
    ):
        agent_group = [x for (i, x) in enumerate(agents) if agent_groups[i] == group]

        local_team_agent_indices = dict(
            [
                (team, [j for (j, a) in enumerate(agent_group) if a[1] == team])
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
            solution = self.solve_tapf([a], {a[0]: 0}, team_goals)
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
                k_solved = len(agents)
                self.max_k_solved = max(k_solved, self.max_k_solved)
                context = None
                if self.config.conflict_avoidance and k_solved < self.k:
                    other_agents = [
                        i for i in range(self.k) if agent_groups[i] != merged_group
                    ]
                    other_sum = 0
                    for group in unique_groups:
                        if group != merged_group and group != conflict_group:
                            other_sum += group_sic[group]
                    context = IDContext(other_sum, other_agents, agent_paths, lens)

                group_sol = self.solve_tapf_group(
                    agent_groups[conflicting_pair[0]],
                    agent_groups,
                    all_agents,
                    team_agent_indices,
                    team_goals,
                    context,
                    0,
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
        return ICTSolution(final_path, sum(group_sic.values()))

    def solve_mapf_group(
        self,
        group: int,
        agent_groups: List[int],
        matching: List[Tuple[CompactLocation, CompactLocation]],
        context: Optional[IDContext],
        lower_sic_bound=0,
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
                lower_sic_bound = group_sic[merged_group] + group_sic[conflict_group]
                group_sic[conflict_group] = 0
                agent_groups = merge_groups(agent_groups, merged_group, conflict_group)
                agents = [i for i in range(self.k) if agent_groups[i] == merged_group]

                k_solved = len(agents)
                self.max_k_solved = max(k_solved, self.max_k_solved)
                context = None
                if self.config.conflict_avoidance and k_solved < self.k:
                    other_agents = [
                        i for i in range(self.k) if agent_groups[i] != merged_group
                    ]
                    other_sum = 0
                    for group in unique_groups:
                        if group != merged_group and group != conflict_group:
                            other_sum += group_sic[group]
                    context = IDContext(other_sum, other_agents, agent_paths, lens)
                group_sol = self.solve_mapf_group(
                    agent_groups[conflicting_pair[0]],
                    agent_groups,
                    matching,
                    context,
                    0,
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
        return ICTSolution(final_path, sum(group_sic.values()))
