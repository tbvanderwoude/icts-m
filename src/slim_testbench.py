from multiprocessing import Pool

from func_timeout import func_timeout, FunctionTimedOut
from mapfmclient.functime import time_fun
from tqdm import tqdm


class TimeoutSolver:
    def __init__(self, solver, timeout):
        self.solver = solver
        self.timeout = timeout

    def __call__(self, current_problem):
        try:
            sol = func_timeout(self.timeout / 1000, self.solver, args=(current_problem,))

        except FunctionTimedOut:
            sol = None
        # except Exception as e:
        #     print(f"An error occurred while running: {e}")
        #     return None
        return sol


# https://stackoverflow.com/questions/62186218/python-multiprocessing-attributeerror-cant-pickle-local-object
# I wonder why this approach (like rust closures) is not default...

class TimingFunction:
    def __init__(self, solve_func):
        self.solve_func = solve_func

    def __call__(self, current_problem):
        return current_problem, *time_fun(current_problem, self.solve_func)


class TestBench:
    def __init__(self, cores=-1, timeout=4000):
        self.cores = cores
        self.timeout = timeout

    def solve_problems(self, solver, problem_list):
        self.solver = solver
        if self.timeout:
            solve_func = TimeoutSolver(solver, self.timeout)
        else:
            solve_func = self.solver
        time_func = TimingFunction(solve_func)
        if self.cores == 1:
            solutions = list(tqdm(
                map(time_func, problem_list),
                total=len(problem_list)
            ))
        elif self.cores == -1:
            with Pool() as p:
                solutions = list(tqdm(
                    p.imap(time_func, problem_list),
                    total=len(problem_list)
                ))
        else:
            with Pool(self.cores) as p:
                solutions = list(tqdm(
                    p.imap(time_func, problem_list),
                    total=len(problem_list)
                ))
        return solutions
