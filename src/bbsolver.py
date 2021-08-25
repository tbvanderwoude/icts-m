from mapf_branch_and_bound.bbsolver import solve_bb
from mapfmclient import Problem
from ictsm.solver import Solver
from ictsm.solver_config import SolverConfig

def solve_bb_api(problem: Problem):
    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=False,
        pruned_child_gen=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=True,
        debug=False,
        budget_search=True,
    )
    solver = Solver(config)
    return solve_bb(problem, solver.call_stripped)