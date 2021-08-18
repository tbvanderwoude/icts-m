import heapq
from typing import List, Tuple, Set, Generator

from mapfmclient import Problem

from ictsm.astar import astar
from ictsm.maze import Maze
from branch_and_bound.bbnode import BBNode
from branch_and_bound.assignment_solver import solve_problem
from ictsm.compact_location import MarkedCompactLocation, compact_location
from branch_and_bound.assignment_problem import AssignmentProblem


def evaluate(node: BBNode):
    return node.lower_bound + node.__hash__() % 100

def murty_gen(costs, root):
    ls: List[BBNode] = [root]
    heapq.heapify(ls)
    seen: Set[BBNode] = set()
    while ls:
        n: BBNode = heapq.heappop(ls)
        if n not in seen: # and (not min_cost or n.lower_bound < min_cost):
            seen.add(n)
            if n.is_leaf():
                # c = evaluate(n)
                # print("{} Leaf node with lower-bound {} evaluated to {} (current upper: {})".format(index,n.lower_bound, c,
                #                                                                                  min_cost))
                yield n;
                # index+=1
                # if not min_cost or c < min_cost:
                #     min_cost = c
            else:
                children = n.problem.generate_subproblems()
                if len(children) == 1:
                    heapq.heappush(ls, BBNode(n, children[0], n.lower_bound))
                else:
                    for sub_problem in n.problem.generate_subproblems():
                        sub_cost = solve_problem(costs, sub_problem)
                        heapq.heappush(ls, BBNode(n, sub_problem, sub_cost))

def solve_bb(problem: Problem):
    # translates MAPFM problem to assignment problem (relaxation
    k = len(problem.starts)
    costs = [[0 for _ in range(k)] for _ in range(k)]
    # makes sure that the K teams are numbered without gaps as 0...(K-1)
    reverse_map = enumerate(sorted(set(map(lambda x: x.color, problem.starts))))
    color_map = dict([(sub[1], sub[0]) for sub in reverse_map])
    K = len(color_map)
    # print(color_map)
    print("There are {} agents in {} teams".format(k,K))
    agents: List[MarkedCompactLocation] = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), color_map[marked.color]),
            problem.starts,
        )
    )
    goals: List[MarkedCompactLocation] = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), color_map[marked.color]),
            problem.goals,
        )
    )
    # print(agents,goals)
    maze: Maze = Maze(problem.grid, problem.width, problem.height)
    for (i,(al,ac)) in enumerate(agents):
        for (j,(gl,gc)) in enumerate(goals):
            if ac == gc:
                shortest_path = astar(maze, al, gl)
                c = len(shortest_path) - 1
                costs[i][j] = c
    # print(costs)
    # costs = [
    #     [90, 80, 75, 70, 10, 30],
    #     [35, 85, 55, 65, 50, 10],
    #     [125, 95, 90, 95, 30, 15],
    #     [45, 110, 180, 115, 20, 30],
    #     [50, 120, 95, 115, 10, 5],
    #     [5, 20, 5, 15, 100, 15],
    # ]
    team_id = [x[1] for x in agents]
    team_tasks =[set([g[0] for g in enumerate(goals) if g[1][1] == team]) for team in range(K)]

    # print(team_id,team_tasks)
    root_problem = AssignmentProblem(team_id, team_tasks, K, k, k)
    root_cost = solve_problem(costs, root_problem)
    root = BBNode(None, root_problem, root_cost)
    matching_generator = murty_gen(costs, root)
    for bb_node in matching_generator:
        print(bb_node.problem.assignments,bb_node.lower_bound)
    return None


# if __name__ == "__main__":
#     costs = [
#         [90, 80, 75, 70, 10, 30],
#         [35, 85, 55, 65, 50, 10],
#         [125, 95, 90, 95, 30, 15],
#         [45, 110, 180, 115, 20, 30],
#         [50, 120, 95, 115, 10, 5],
#         [5, 20, 5, 15, 100, 15],
#     ]
#     team_id = [0, 0, 0, 0, 0, 0]
#     team_tasks = [{0, 1, 2, 3, 4, 5}]
#     problem = AssignmentProblem(team_id, team_tasks, K, len(costs), len(costs[0]))
#     root_cost = solve_problem(costs, problem)
#     root = BBNode(None, problem, root_cost)
#     branch_and_bound(costs, root)
