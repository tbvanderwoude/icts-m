import os
import pickle
from typing import List

from mapfmclient import Problem, MarkedLocation
import random
import numpy as np
import shutil

from mapfm.solver import Solver
from mapfm.solver_config import SolverConfig
from parsing.map_parser import MapParser
from mapfm.solve import solve, solve_enum, solve_enum_sorted, solve_enum_sorted_prune_child, solve_pc, \
    solver_from_config
from mapfmclient.test_bench import TestBench


class BenchmarkQueue:
    def __init__(self, name):
        self.name = name
        with open(name, "a"):
            pass

    def get_next(self):
        with open(self.name, "r") as f:
            return f.readline().strip()

    def completed(self):
        with open(self.name, "r") as fin:
            data = fin.read().splitlines(True)
        with open(self.name, "w") as fout:
            fout.writelines(data[1:])

    def add(self, data):
        with open(self.name, "a") as f:
            f.write(data + "\n")


def p_bool(p):
    return random.random() < p


def gen_rand_grid(width, height, p):
    rows = []
    for y in range(height):
        rows.append([int(p_bool(p)) for x in range(width)])
    return rows


def gen_legal_point(taken, grid, width, height):
    point = None
    while not point:
        start_x = random.randrange(width)
        start_y = random.randrange(height)
        if not grid[start_y][start_x] and not (start_x, start_y) in taken:
            point = start_x, start_y
    return point


def gen_agent_goals(grid, width, height, t, k_team):
    starts = []
    goals = []
    taken_starts = set()
    taken_goals = set()
    taken = set()
    for team in range(t):
        for agent in range(k_team):
            start_x, start_y = gen_legal_point(taken, grid, width, height)
            taken_starts.add((start_x, start_y))
            taken.add((start_x, start_y))
            goal_x, goal_y = gen_legal_point(taken, grid, width, height)
            taken_goals.add((goal_x, goal_y))
            taken.add((goal_x, goal_y))
            start = MarkedLocation(team, start_x, start_y)
            goal = MarkedLocation(team, goal_x, goal_y)
            starts.append(start)
            goals.append(goal)
    return starts, goals


def show_problem(grid, width, height, starts, goals):
    for y in range(height):
        s = ""
        for x in range(width):
            if grid[y][x]:
                s += "#"
            else:
                s += " "
        print(s)
    return starts, goals


def gen_problem(width, height, density, t, k_team):
    grid = gen_rand_grid(width, height, density)
    starts, goals = gen_agent_goals(grid, width, height, t, k_team)
    return Problem(grid, width, height, starts, goals)


def gen_problem_random(width, height, density, t, k_team):
    grid = gen_rand_grid(width, height, density)
    starts, goals = gen_agent_goals(grid, width, height, t, k_team)
    return Problem(grid, width, height, starts, goals)


# computes success rate and avg + std run-time
def process_results(solutions):
    return (
        np.array([int(bool(x[1]) and bool(x[1][0])) for x in solutions]).mean(),
        np.array([x[2] for x in solutions]).mean(),
        np.array([x[2] for x in solutions]).std(),
    )


def solve_setting(solver, t, k_team, timeout, samples):
    print("t = {} (k = {})".format(t, t * k_team))
    problems = [gen_problem(20, 20, 0.0, t, k_team) for i in range(samples)]
    bench = TestBench(-1, timeout)
    enum_sols = bench.run(solver, problems)
    return enum_sols


def test_queue(solver, map_parser, timeout, queue: BenchmarkQueue):
    output = solver.name
    task = queue.get_next()
    with open(output, "a") as f:
        f.write(f"task_id,completed,avg,std\n")
    raw_dir = "raw/{}".format(output)
    if os.path.exists(raw_dir):
        shutil.rmtree(raw_dir)
    os.mkdir(raw_dir)
    while task is not None and task != "":
        with open(output, "a") as f:
            problems = map_parser.parse_batch(task)
            bench = TestBench(1, timeout)
            enum_sols = bench.run(solver, problems)
            with open("{}/{}".format(raw_dir,task),"wb+") as raw:
                pickle.dump(enum_sols, raw)
            res, mean, std = process_results(enum_sols)
            f.write(f"{task}, {res}, {mean}, {std}\n")
            print(f"{task}: {res} with average {mean}s and deviation: {std}\n")
            queue.completed()
            task = queue.get_next()

def compare_configs(configs: List[SolverConfig]):
    map_root = "maps"
    map_parser = MapParser(map_root)
    for (i,config) in enumerate(configs):
        os.system("cp /dev/null results_{}.txt; cp full_queue.txt queue.txt".format(i))
        test_queue(ConfiguredSolver(config), map_parser, 100, BenchmarkQueue("queue.txt"))

class ConfiguredSolver:
    def __init__(self,config: SolverConfig):
        self.config = config
        self.name = config.name

    def __call__(self,problem):
        return Solver(self.config,problem)()

    def __name__(self):
        return self.name

if __name__ == "__main__":
    configs = [
        SolverConfig(
        name = "enum_no_cp",
        combs=3,
        prune=True,
        enhanced=True,
        pruned_child_gen=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        debug=False,
        sort_matchings=True,
        ),
        SolverConfig(
            name="enum_cp",
            combs=2,
            prune=True,
            enhanced=False,
            pruned_child_gen=True,
            id=True,
            conflict_avoidance=True,
            enumerative=True,
            debug=False,
            sort_matchings=True,
        )
    ]
    compare_configs(configs)