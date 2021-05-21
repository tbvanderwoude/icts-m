from typing import List, Tuple, Optional

from mapfmclient import Problem, Solution

from mapfm.solver import SolverConfig, Solver


def solve_api(problem: Problem) -> Solution:
    return solve(problem)[0]


def solve_api_enum(problem: Problem) -> Solution:
    return solve_enum_sorted(problem)[0]


def solve(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=2,
        prune=True,
        enhanced=True,
        id=False,
        conflict_avoidance=True,
        enumerative=False,
        debug=True,
    )
    return Solver(config, problem)()


def solve_enum_sorted(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=True,
        debug=True,
    )
    return Solver(config, problem)()


def solve_enum(problem: Problem) -> Tuple[Optional[Solution], List[int], int]:
    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=False,
        debug=False,
    )
    return Solver(config, problem)()