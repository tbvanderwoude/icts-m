from copy import copy, deepcopy

class AssignmentProblem(object):
    def __init__(self, team_ids,team_tasks, num_teams,num_workers, num_tasks, assigned_workers = 0, assignments = []):
        self.team_ids = team_ids
        self.num_teams = num_teams
        self.num_workers = num_workers
        self.num_tasks = num_tasks
        self.team_tasks = team_tasks
        self.assigned_workers = assigned_workers
        self.assignments = assignments
        self.cost = 0

    def fully_assigned(self):
        return self.assigned_workers >= self.num_workers

    def __hash__(self):
        return hash(tuple(self.assignments))

    def generate_subproblems(self):
        if self.fully_assigned():
            return None
        else:
            subproblems = []
            n_num_teams = self.num_teams + 1
            for task in self.team_tasks[self.team_ids[self.assigned_workers]]:
                n_team_ids = copy(self.team_ids)
                n_team_tasks = deepcopy(self.team_tasks)
                task_team = self.team_ids[self.assigned_workers]
                n_team_tasks[task_team].remove(task)
                n_team_tasks.append({task})
                n_team_ids[self.assigned_workers] = n_num_teams-1
                n_assignments = copy(self.assignments)
                n_assigned_workers = self.assigned_workers + 1
                n_assignments.append(task)
                n_subproblem = AssignmentProblem(n_team_ids, n_team_tasks, n_num_teams, self.num_workers, self.num_tasks,
                                                 assigned_workers=n_assigned_workers, assignments=n_assignments)
                subproblems.append(n_subproblem)
            return subproblems

    def __str__(self):
        return '{self.assignments},{self.team_tasks}'.format(self=self)