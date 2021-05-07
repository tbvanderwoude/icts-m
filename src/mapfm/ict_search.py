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


class ICTSearcher:
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
        self.budget = budget

    def calculate_upper_bound_cost(self, k: int):
        return (k ** 2) * self.open_spaces

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

    def search(
        self,
        subproblems: List[Tuple[CompactLocation, CompactLocation]],
        root: Tuple[int, ...],
        context: Optional[IDContext] = None,
    ) -> Optional[ICTSolution]:
        k = len(subproblems)
        if self.budget is None:
            budget = self.calculate_upper_bound_cost(k)
        else:
            budget = self.budget
        frontier = deque()
        frontier.append(root)
        visited = set()
        mdd_cache = dict()
        # print(k, budget, root)
        while frontier:
            node = frontier.popleft()
            if sum(node) <= budget and not node in visited:
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
                                subproblems[i][1],
                                c,
                                mdd_cache[(i, c - 1)],
                            )
                        else:
                            mdd_cache[(i, c)] = MDD(
                                self.maze, i, subproblems[i][0], subproblems[i][1], c
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
