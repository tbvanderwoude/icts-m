from copy import copy, deepcopy
from typing import Optional
from ortools.linear_solver import pywraplp

from problem import Problem

solver = pywraplp.Solver.CreateSolver('SCIP')

from ortools.linear_solver import pywraplp

def solve_problem(costs, problem: Problem):
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
        print('Total cost = ', solver.Objective().Value(), '\n')
        for i in range(problem.num_workers):
            for j in range(problem.num_tasks):
                # Test if x[i,j] is 1 (with tolerance for floating point arithmetic).
                if x[i, j].solution_value() > 0.5:
                    print('Worker %d assigned to task %d.  Cost = %d' %
                          (i, j, costs[i][j]))


# class BBNode(object):
#     def __init__(self):
#         self.parent: Optional[BBNode] =
#         self.bound: int =
#
#     def __hash__(self):
#         return hash(self.loc)
#
#     def is_root(self):
#         return self.parent is None
#
#     def is_goal(self, goal: CompactLocation) -> bool:
#         return self.loc == goal
#
#     def get_directions(self):
#         if self.is_root():
#             return [self.loc]
#         else:
#             par_dirs = self.parent.get_directions()
#             par_dirs.append(self.loc)
#             return par_dirs

if __name__ == "__main__":
    costs = [
        [90, 80, 75, 70],
        [35, 85, 55, 65],
        [125, 95, 90, 95],
        [45, 110, 95, 115],
    ]
    team_id = [0, 0, 1, 2]
    team_tasks = [{0, 1}, {2}, {3}]
    problem = Problem(team_id, 3,team_tasks, 4, 4)
    solve_problem(costs, problem)
