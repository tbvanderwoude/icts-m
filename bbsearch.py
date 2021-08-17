import heapq
from typing import List, Tuple, Set

from bbnode import BBNode
from problem import Problem
from solver import solve_problem

def evaluate(node: BBNode):
    return node.lower_bound + node.__hash__() % 100

def branch_and_bound(costs,root: BBNode) -> List[Tuple[int, int]]:
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
                print("Leaf node with lower-bound {} evaluated to {} (current upper: {})".format(n.lower_bound,c,min_cost))
                if not min_cost or c < min_cost:
                    min_cost = c
                # return list(map(lambda loc: expand_location(loc), n.get_directions()))
            else:
                children = n.problem.generate_subproblems()
                if len(children) == 1:
                    heapq.heappush(ls, BBNode(n, children[0], n.lower_bound))
                else:
                    for sub_problem in n.problem.generate_subproblems():
                        sub_cost = solve_problem(costs,sub_problem)
                        heapq.heappush(ls, BBNode(n, sub_problem, sub_cost))
    return []


if __name__ == "__main__":
    costs = [
        [90, 80, 75, 70,10,30],
        [35, 85, 55, 65,50,10],
        [125, 95, 90, 95,30,15],
        [45, 110, 180, 115,20,30],
        [50, 120, 95, 115, 10,5],
        [5, 20, 5, 15, 100,15],
    ]
    team_id = [0, 0, 0, 0,0,0]
    team_tasks = [{0, 1, 2,3,4,5}]
    problem = Problem(team_id, team_tasks, len(team_tasks), len(costs), len(costs[0]))
    root_cost = solve_problem(costs,problem)
    root = BBNode(None,problem,root_cost)
    branch_and_bound(costs,root)
    # solve_problem(costs, problem)
    # for subproblem in problem.generate_subproblems():
    #     print(subproblem)
    #     solve_problem(costs, subproblem)