from typing import List, Tuple, Optional

from mapfmclient import Problem, Solution

from mapfm.solver import Solver
from mapfm.solver_config import SolverConfig


def solve_api(problem: Problem) -> Solution:
    return solve(problem)[0]


def solve_api_enum(problem: Problem) -> Solution:
    return solve_enum_sorted(problem)[0]


def solve(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=2,
        prune=True,
        enhanced=True,
        pruned_child_gen=False,
        id=True,
        conflict_avoidance=True,
        enumerative=False,
        debug=False,
    )
    return Solver(config, problem)()


def solve_enum_sorted(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=True,
        pruned_child_gen=False,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=True,
        debug=False,
    )
    return Solver(config, problem)()


def solve_enum(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=2,
        prune=True,
        enhanced=True,
        pruned_child_gen=False,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=False,
        debug=False,
    )
    return Solver(config, problem)()