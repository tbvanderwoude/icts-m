from copy import copy, deepcopy

import solver


class Problem(object):
    def __init__(self, team_ids,team_tasks, num_teams,num_workers, num_tasks, assigned_workers = 0, assignments = []):
        self.team_ids = team_ids
        self.num_teams = num_teams
        self.num_workers = num_workers
        self.num_tasks = num_tasks
        self.team_tasks = team_tasks
        self.assigned_workers = assigned_workers
        self.assignments = assignments

    def generate_subproblems(self):
        if self.assigned_workers >= self.num_workers:
            return None
        else:
            subproblems = []
            n_num_teams = self.num_teams + 1
            for task in self.team_tasks[self.assigned_workers]:
                n_team_ids = copy(self.team_ids)
                n_team_tasks = deepcopy(self.team_tasks)
                n_team_tasks[self.team_ids[self.assigned_workers]].remove(task)
                n_team_tasks.append({task})
                n_team_ids[self.assigned_workers] = n_num_teams-1
                n_assignments = copy(self.assignments)
                n_assigned_workers = self.assigned_workers + 1
                n_assignments.append(task)
                subproblems.append(Problem(n_team_ids,n_team_tasks,n_num_teams,self.num_workers,self.num_tasks,assigned_workers=n_assigned_workers,assignments=n_assignments))
            return subproblems

    def __str__(self):
        return '{self.assignments},{self.team_tasks}'.format(self=self)


if __name__ == "__main__":
    costs = [
        [90, 80, 75, 70],
        [35, 85, 55, 65],
        [125, 95, 90, 95],
        [45, 110, 95, 115],
    ]
    team_id = [0, 0, 1, 2]
    team_tasks = [{0, 1}, {2}, {3}]
    problem = Problem(team_id, team_tasks, 3, 4, 4)
    solver.solve_problem(costs, problem)
    for subproblem in problem.generate_subproblems():
        print(subproblem)
        solver.solve_problem(costs, subproblem)