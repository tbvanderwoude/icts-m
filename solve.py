from copy import copy
from typing import List, Tuple

from mapfmclient import Problem, Solution

from astar import astar
from compact_location import compact_location, expand_location
from ict_search import ict_search
from maze import Maze


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


def solve(problem: Problem) -> Solution:
    combs = 2
    prune = True
    enhanced = True
    maze: Maze = Maze(problem.grid, problem.width, problem.height)
    paths: List[List[Tuple[int, int]]] = []
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
        subproblems = []
        root_list = []
        for (start, goal) in matching:
            subproblems.append((start, goal))
            shortest = astar(maze, start, goal)
            assert len(shortest) > 0
            root_list.append(len(shortest) - 1)
        root = tuple(root_list)
        sol = ict_search(maze, combs, prune, enhanced, subproblems, root)
        stripped_sol = list(map(lambda x: x[0], sol))
        sic = len(stripped_sol)
        if not min_sic or min_sic > sic:
            min_sic = sic
            min_sol = stripped_sol
    # print(min_sic)
    subsols = list(zip(*min_sol))
    for subsol in subsols:
        paths.append(list(map(lambda loc: expand_location(loc), subsol)))
    return Solution.from_paths(paths)
