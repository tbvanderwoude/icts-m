from collections import deque
from itertools import combinations
from typing import List, Tuple, Optional

from mapfm.compact_location import CompactLocation
from mapfm.maze import Maze
from mapfm.mdd import MDD
from mapfm.mdd_search import (
    JointSolution,
    seek_solution_in_joint_mdd,
    JointTimedSolution,
)


def find_number_of_open_spaces(maze: Maze):
    return sum([sum([x ^ 1 for x in row]) for row in maze.grid])


def calculate_upper_bound_cost(agents: int, maze: Maze):
    number_of_open_spaces = find_number_of_open_spaces(maze)
    return (agents ** 2) * number_of_open_spaces


def ict_search(
    maze: Maze,
    combs,
    prune,
    enhanced,
    subproblems: List[Tuple[CompactLocation, CompactLocation]],
    root: Tuple[int, ...],
) -> Optional[JointSolution]:
    frontier = deque()
    frontier.append(root)
    visited = set()
    mdd_cache = dict()

    k = len(subproblems)
    upper = calculate_upper_bound_cost(k, maze)
    # print(k, upper, root)
    while frontier:
        node = frontier.popleft()
        if sum(node) <= upper and not node in visited:
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
                            maze,
                            i,
                            subproblems[i][0],
                            subproblems[i][1],
                            c,
                            mdd_cache[(i, c - 1)],
                        )
                    else:
                        mdd_cache[(i, c)] = MDD(
                            maze, i, subproblems[i][0], subproblems[i][1], c
                        )
                mdds.append(mdd_cache[(i, c)])
            if (
                not prune
                or k <= combs
                or check_combinations(combs, mdds, k, enhanced, accumulator)
            ):
                solution: JointTimedSolution = seek_solution_in_joint_mdd(mdds, True)
                if solution:
                    return list(map(lambda x: x[0], solution))

            for (i, p, c) in accumulator:
                mdds[i].mdd[p].add(c)
    return []


def check_combinations(
    combs, mdds: List[MDD], k: int, enhanced: bool = True, accumulator: List = []
):
    for c in combinations(range(k), combs):
        if not seek_solution_in_joint_mdd(
            [mdds[i] for i in c], False, enhanced, accumulator
        ):
            return False
    return True
