from typing import Optional

from assignment_problem import AssignmentProblem


class BBNode(object):
    def __init__(self, parent, problem: AssignmentProblem, lower_bound: int):
        self.parent: Optional[BBNode] = parent
        self.problem: AssignmentProblem = problem
        self.lower_bound: int = lower_bound
        self.children = None

    def __hash__(self):
        return hash(self.problem)

    def is_root(self):
        return self.parent is None

    def is_leaf(self) -> bool:
        return self.problem.fully_assigned()

    def __eq__(self, other) -> bool:
        return self.lower_bound == other.lower_bound

    def __ne__(self, other) -> bool:
        return self.lower_bound != other.lower_bound

    def __ge__(self, other) -> bool:
        return self.lower_bound >= other.lower_bound

    def __lt__(self, other) -> bool:
        return self.lower_bound < other.lower_bound

    def __gt__(self, other) -> bool:
        return self.lower_bound > other.lower_bound

    def __le__(self, other) -> bool:
        return self.lower_bound <= other.lower_bound