import heapq
from typing import List, Tuple, Set

from bbnode import BBNode
from problem import Problem
from solver import solve_problem

def evaluate(node: BBNode):
    return node.lower_bound + node.__hash__() % 400

def astar(costs,root: BBNode) -> List[Tuple[int, int]]:
    ls: List[BBNode] = [root]
    heapq.heapify(ls)
    seen: Set[BBNode] = set()
    min_cost = None
    while ls:
        n: BBNode = heapq.heappop(ls)
        if n not in seen and (not min_cost or n.lower_bound < min_cost):
            seen.add(n)
            if n.is_leaf():
                c = evaluate(n)
                print("Leaf node with lower-bound {} evaluated to {}".format(n.lower_bound,c))
                if not min_cost or c < min_cost:
                    min_cost = c
                # return list(map(lambda loc: expand_location(loc), n.get_directions()))
            else:
                for sub_problem in n.problem.generate_subproblems():
                    sub_cost = solve_problem(costs,sub_problem)
                    heapq.heappush(ls, BBNode(n, sub_problem, sub_cost))
    return []


if __name__ == "__main__":
    costs = [
        [90, 80, 75, 70],
        [35, 85, 55, 65],
        [125, 95, 90, 95],
        [45, 110, 95, 115],
    ]
    team_id = [0, 0, 0, 0]
    team_tasks = [{0, 1, 2,3}]
    problem = Problem(team_id, team_tasks, len(team_tasks), len(costs), len(costs[0]))
    root_cost = solve_problem(costs,problem)
    root = BBNode(None,problem,root_cost)
    astar(costs,root)
    # solve_problem(costs, problem)
    # for subproblem in problem.generate_subproblems():
    #     print(subproblem)
    #     solve_problem(costs, subproblem)