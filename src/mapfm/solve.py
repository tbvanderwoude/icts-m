from copy import copy
from typing import List, Tuple

from mapfmclient import Problem, Solution

from mapfm.astar import astar
from mapfm.compact_location import compact_location, expand_location, CompactLocation
from mapfm.conflicts import is_invalid_move, find_conflict
from mapfm.ict_search import ict_search
from mapfm.maze import Maze


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


def solve_matching(
    maze: Maze,
    combs: int,
    prune: bool,
    enhanced: bool,
    matching: List[Tuple[CompactLocation, CompactLocation]],
):
    subproblems = []
    root_list = []
    for (start, goal) in matching:
        subproblems.append((start, goal))
        shortest = astar(maze, start, goal)
        assert len(shortest) > 0
        root_list.append(len(shortest) - 1)
    root = tuple(root_list)
    return ict_search(maze, combs, prune, enhanced, subproblems, root)


def solve_group(
    group: int,
    agent_groups: List[int],
    maze: Maze,
    combs: int,
    prune: bool,
    enhanced: bool,
    matching: List[Tuple[CompactLocation, CompactLocation]]
):
    sub_matching = [x for (i, x) in enumerate(matching) if agent_groups[i] == group]
    return list(zip(*solve_matching(maze, combs, prune, enhanced, sub_matching)))

def index_path(path, i, l):
    if i < l:
        return path[i]
    else:
        return path[-1]


def merge_groups(agent_groups, group_i, group_j):
    new_agent_groups = []
    for agent_group in agent_groups:
        if agent_group == group_j:
            new_agent_groups.append(group_i)
        else:
            new_agent_groups.append(agent_group)
    return new_agent_groups


def solve_mapf_with_id(
    maze: Maze,
    combs: int,
    prune: bool,
    enhanced: bool,
    matching: List[Tuple[CompactLocation, CompactLocation]],
    k: int,
):
    agent_groups = list(range(k))
    agent_paths: List[List[Tuple[CompactLocation]]] = []
    for (i,match) in enumerate(matching):
        agent_paths.append(list(map(lambda x: x[0],solve_matching(maze, combs, prune, enhanced, [match]))))
    # print(agent_paths)
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
            node = tuple([index_path(agent_paths[i], t, lens[i]) for i in range(k)])
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
            agent_groups = merge_groups(
                agent_groups, merged_group, conflict_group
            )
            agents = [i for i in range(k) if agent_groups[i] == merged_group]
            k_solved = len(agents)
            sol = solve_group(agent_groups[conflicting_pair[0]],agent_groups,maze,combs,prune,enhanced,matching)
            for (i,agent) in enumerate(agents):
                agent_paths[agent] = sol[i]
            kprime = max(kprime,k_solved)
        else:
            break
    print("k': {}".format(kprime))
    return final_path


def solve(problem: Problem) -> Solution:
    combs = 2
    prune = True
    enhanced = True
    id = True
    maze: Maze = Maze(problem.grid, problem.width, problem.height)
    paths: List[List[Tuple[int, int]]] = []
    k = len(problem.starts)
    agents = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), marked.color),
            problem.starts,
        )
    )
    goals = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), marked.color),
            problem.goals,
        )
    )
    matchings = enumerate_matchings(agents, goals)
    min_sic = None
    min_sol = None
    for matching in matchings:
        if id:
            sol = solve_mapf_with_id(maze, combs, prune, enhanced, matching, k)
        else:
            sol = solve_matching(maze, combs, prune, enhanced, matching)
        sic = len(sol)
        if not min_sic or min_sic > sic:
            min_sic = sic
            min_sol = sol
    # print(min_sic)
    subsols = list(zip(*min_sol))
    for subsol in subsols:
        paths.append(list(map(lambda loc: expand_location(loc), subsol)))
    return Solution.from_paths(paths)
