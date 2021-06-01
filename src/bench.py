import os
import pickle
from typing import List

import numpy as np
import shutil
import pathlib

from ictsm.benchmark_queue import BenchmarkQueue
from ictsm.solver import Solver
from ictsm.solver_config import SolverConfig
from mapfmclient.parser import MapParser
from mapfmclient.test_bench import TestBench

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

# computes success rate and avg + std run-time
def process_results(solutions):
    return (
        np.array([int(bool(x[1]) and bool(x[1][0])) for x in solutions]).mean(),
        np.array([x[2] for x in solutions]).mean(),
        np.array([x[2] for x in solutions]).std(),
    )


def test_queue(solver, map_parser, timeout,queue: BenchmarkQueue, num: int = 200):
    num = min(num,200)
    output = pathlib.Path(solver.name)
    task = queue.get_next()
    if output.exists():
        output.unlink()
    with output.open("a") as f:
        f.write(f"task_id,completed,avg,std\n")
    raw_dir = pathlib.Path("raw/{}".format(output))
    if raw_dir.exists():
        shutil.rmtree(raw_dir)
    raw_dir.mkdir(parents=True)
    if os.path.exists(raw_dir):
        shutil.rmtree(raw_dir)
    os.mkdir(raw_dir)
    while task is not None and task != "":
        with open(output, "a") as f:
            problems = map_parser.parse_batch(task)[:num]
            bench = TestBench(-1, timeout)
            enum_sols = bench.run(solver, problems)
            with (raw_dir / task).open("wb+") as raw:
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
        os.system("cp full_queue.txt queue.txt")
        test_queue(ConfiguredSolver(config), map_parser, 30000, BenchmarkQueue("queue.txt"),50)

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
        name = "exh. icts",
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
        name="icts-m",
        combs=3,
        prune=True,
        enhanced=True,
        pruned_child_gen=True,
        id=False,
        conflict_avoidance=False,
        enumerative=False,
        debug=False,
        sort_matchings=False,
        )
    ]
    compare_configs(configs)