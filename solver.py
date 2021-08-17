from copy import copy, deepcopy
from typing import Optional
from ortools.linear_solver import pywraplp

from problem import Problem

solver = pywraplp.Solver.CreateSolver('SCIP')

from ortools.linear_solver import pywraplp

def solve_problem(costs, problem: Problem) -> int:
    x = {}
    for i in range(problem.num_workers):
        for j in range(problem.num_tasks):
            x[i, j] = solver.IntVar(0, 1, '')
    # Each worker is assigned to exactly 1 task.
    for i in range(problem.num_workers):
        solver.Add(solver.Sum([x[i, j] for j in problem.team_tasks[problem.team_ids[i]]]) == 1)

    # Each task is assigned to exactly one worker.
    for j in range(problem.num_tasks):
        solver.Add(solver.Sum([x[i, j] for i in range(problem.num_workers)]) == 1)

    objective_terms = []
    for i in range(problem.num_workers):
        for j in range(problem.num_tasks):
            objective_terms.append(costs[i][j] * x[i, j])
    solver.Minimize(solver.Sum(objective_terms))

    status = solver.Solve()
    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        total_cost = solver.Objective().Value()
        # print('Total cost = ', total_cost, '\n')
        # for i in range(problem.num_workers):
        #     for j in range(problem.num_tasks):
        #         # Test if x[i,j] is 1 (with tolerance for floating point arithmetic).
        #         if x[i, j].solution_value() > 0.5:
        #             print('Worker %d assigned to task %d.  Cost = %d' %
        #                   (i, j, costs[i][j]))
        return total_cost


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
