import heapq
from typing import List, Tuple, Set

from mapfmclient import Problem

from ictsm.astar import astar
from ictsm.maze import Maze
from branch_and_bound.bbnode import BBNode
from branch_and_bound.assignment_solver import solve_problem
from ictsm.compact_location import MarkedCompactLocation, compact_location


def evaluate(node: BBNode):
    return node.lower_bound + node.__hash__() % 100

def solve_bb(problem: Problem):
    print(problem)
    print(problem.starts)
    k = len(problem.starts)
    costs = [[0 for j in range(k)] for i in range(k)]
    # paths: List[List[Tuple[int, int]]] = []
    agents: List[MarkedCompactLocation] = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), marked.color),
            problem.starts,
        )
    )
    goals: List[MarkedCompactLocation] = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), marked.color),
            problem.goals,
        )
    )
    maze: Maze = Maze(problem.grid, problem.width, problem.height)
    for (i,(al,ac)) in enumerate(agents):
        for (j,(gl,gc)) in enumerate(goals):
            if ac == gc:
                shortest_path = astar(maze, al, gl)
                c = len(shortest_path) - 1
                costs[i][j] = c

    print(costs)
    return None
    # costs = [
    #     [90, 80, 75, 70, 10, 30],
    #     [35, 85, 55, 65, 50, 10],
    #     [125, 95, 90, 95, 30, 15],
    #     [45, 110, 180, 115, 20, 30],
    #     [50, 120, 95, 115, 10, 5],
    #     [5, 20, 5, 15, 100, 15],
    # ]
    # team_id = [0, 0, 0, 0, 0, 0]
    # team_tasks = [{0, 1, 2, 3, 4, 5}]

    # problem = Problem(team_id, team_tasks, len(team_tasks), len(costs), len(costs[0]))
    # root_cost = solve_problem(costs, problem)
    # root = BBNode(None, problem, root_cost)
    #
    # ls: List[BBNode] = [root]
    # heapq.heapify(ls)
    # seen: Set[BBNode] = set()
    # min_cost = None
    # index = 0
    # while ls:
    #     n: BBNode = heapq.heappop(ls)
    #     if n not in seen and (not min_cost or n.lower_bound < min_cost):
    #         seen.add(n)
    #         if n.is_leaf():
    #             c = evaluate(n)
    #             print("{} Leaf node with lower-bound {} evaluated to {} (current upper: {})".format(index,n.lower_bound, c,
    #                                                                                              min_cost))
    #             index+=1
    #             if not min_cost or c < min_cost:
    #                 min_cost = c
    #         else:
    #             children = n.problem.generate_subproblems()
    #             if len(children) == 1:
    #                 heapq.heappush(ls, BBNode(n, children[0], n.lower_bound))
    #             else:
    #                 for sub_problem in n.problem.generate_subproblems():
    #                     sub_cost = solve_problem(costs, sub_problem)
    #                     heapq.heappush(ls, BBNode(n, sub_problem, sub_cost))
    # return []


if __name__ == "__main__":
    costs = [
        [90, 80, 75, 70, 10, 30],
        [35, 85, 55, 65, 50, 10],
        [125, 95, 90, 95, 30, 15],
        [45, 110, 180, 115, 20, 30],
        [50, 120, 95, 115, 10, 5],
        [5, 20, 5, 15, 100, 15],
    ]
    team_id = [0, 0, 0, 0, 0, 0]
    team_tasks = [{0, 1, 2, 3, 4, 5}]
    problem = Problem(team_id, team_tasks, len(team_tasks), len(costs), len(costs[0]))
    root_cost = solve_problem(costs, problem)
    root = BBNode(None, problem, root_cost)
    branch_and_bound(costs, root)
