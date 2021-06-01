from typing import List, Tuple, Optional

from mapfmclient import Problem, Solution

from ictsm.solver import Solver
from ictsm.solver_config import SolverConfig


def solve_api(problem: Problem) -> Solution:
    return solve(problem)[0]


def solve_api_enum(problem: Problem) -> Solution:
    return solve_enum_sorted(problem)[0]


def solve(problem: Problem) -> Tuple[Optional[Solution], List[int], int,Optional[int]]:
    config = SolverConfig(
        combs=2,
        prune=True,
        enhanced=False,
        pruned_child_gen=False,
        id=True,
        conflict_avoidance=True,
        enumerative=False,
        debug=False,
    )
    return Solver(config, problem)()


def solver_from_config(config: SolverConfig):
    return lambda problem: Solver(config, problem)()


def solve_pc(problem: Problem) -> Tuple[Optional[Solution], List[int], int,Optional[int]]:
    config = SolverConfig(
        combs=2,
        prune=True,
        enhanced=False,
        pruned_child_gen=True,
        id=True,
        conflict_avoidance=True,
        enumerative=False,
        debug=False,
    )
    return Solver(config, problem)()


def solve_enum_sorted(problem: Problem) -> Tuple[Optional[Solution], List[int], int,Optional[int]]:
    config = SolverConfig(
        combs=2,
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


def solve_enum_sorted_prune_child(problem: Problem) -> Tuple[Optional[Solution], List[int], int,Optional[int]]:
    config = SolverConfig(
        combs=2,
        prune=True,
        enhanced=True,
        pruned_child_gen=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=True,
        debug=False,
    )
    return Solver(config, problem)()


def solve_enum(problem: Problem) -> Tuple[Optional[Solution], List[int], int,Optional[int]]:
    config = SolverConfig(
        combs=2,
        prune=True,
        enhanced=False,
        pruned_child_gen=False,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=False,
        debug=False,
    )
    return Solver(config, problem)()
