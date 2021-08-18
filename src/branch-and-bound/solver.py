from copy import copy, deepcopy
from typing import Optional
from ortools.linear_solver import pywraplp

from problem import Problem
from ortools.graph import pywrapgraph
from ortools.linear_solver import pywraplp

def solve_problem(costs, problem: Problem) -> int:
    # x = {}
    rows = len(costs)
    cols = len(costs[0])

    assignment = pywrapgraph.LinearSumAssignment()
    for worker in range(rows):
        for task in problem.team_tasks[problem.team_ids[worker]]:
            if costs[worker][task]:
                assignment.AddArcWithCost(worker, task, costs[worker][task])
    solve_status = assignment.Solve()
    if solve_status == pywraplp.Solver.OPTIMAL:
        total_cost = assignment.OptimalCost()
        # print('Total cost = ', assignment.OptimalCost())
        # print()
        # for i in range(0, assignment.NumNodes()):
        #     print('Worker %d assigned to task %d.  Cost = %d' % (
        #         i,
        #         assignment.RightMate(i),
        #         assignment.AssignmentCost(i)))
        return total_cost
    elif solve_status == assignment.INFEASIBLE:
        print('No assignment is possible.')
    elif solve_status == assignment.POSSIBLE_OVERFLOW:
        print('Some input costs are too large and may cause an integer overflow.')



if __name__ == "__main__":
    costs = [
        [90, 80, 75, 70],
        [35, 85, 55, 65],
        [125, 95, 90, 95],
        [45, 110, 95, 115],
    ]
    team_id = [0, 0, 1, 2]
    team_tasks = [{0, 1}, {2},{3}]
    problem = Problem(team_id, team_tasks, 3, 4, 4)
    solve_problem(costs, problem)
    for subproblem in problem.generate_subproblems():
        print(subproblem)
        solve_problem(costs, subproblem)
