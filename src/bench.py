import os
import pickle
import re
from collections import defaultdict
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


def test_queue(solver, map_parser, timeout,queue: BenchmarkQueue, cores: int = -1,num: int = 200):
    num = min(num,200)
    file_name = solver.name
    output_file_path = pathlib.Path("raw/{}.txt".format(file_name))
    if output_file_path.exists():
        output_file_path.unlink()
    with output_file_path.open("a") as f:
        f.write(f"task_id,completed,avg,std\n")
    raw_dir = pathlib.Path("raw/{}".format(file_name))
    if raw_dir.exists():
        shutil.rmtree(raw_dir)
    raw_dir.mkdir(parents=True)
    if os.path.exists(raw_dir):
        shutil.rmtree(raw_dir)
    os.mkdir(raw_dir)
    bench = TestBench(cores, timeout)
    unsolved = set()

    task = queue.get_next()
    while task is not None and task != "":
        with open(output_file_path, "a") as f:
            m = re.match('([A-Za-z0-9]*)-20x20-A(\d+)_T(\d+)', task)
            map_type = m.group(1)
            agents = int(m.group(2))
            teams = int(m.group(3))
            if not (map_type,teams) in unsolved:
                problems = map_parser.parse_batch(task)[:num]
                enum_sols = bench.run(solver, problems)
                with (raw_dir / task).open("wb+") as raw:
                    pickle.dump(enum_sols, raw)
                res, mean, std = process_results(enum_sols)
                if res == 0.0:
                    print("SOLVED NONE")
                    unsolved.add((map_type,teams))
                f.write(f"{task}, {res}, {mean}, {std}\n")
                print(f"{task}: {res} with average {mean}s and deviation: {std}\n")
            queue.completed()
            task = queue.get_next()

def compare_configs(configs: List[SolverConfig]):
    map_root = "maps"
    map_parser = MapParser(map_root)
    for (i,config) in enumerate(configs):
        os.system("cp full_queue.txt queue.txt")
        test_queue(ConfiguredSolver(config), map_parser, 120000, BenchmarkQueue("queue.txt"),-1,50)

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
        # SolverConfig(
        #     name="(exh-sort)-(enhanced)-(no-id)",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=False,
        #     id=True,
        #     conflict_avoidance=True,
        #     enumerative=True,
        #     debug=False,
        #     sort_matchings=True,
        # ),
        # SolverConfig(
        #     name="(exh)-(enhanced)-(no-id)",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=False,
        #     id=False,
        #     conflict_avoidance=True,
        #     enumerative=True,
        #     debug=False,
        #     sort_matchings=False,
        # ),
        # SolverConfig(
        #     name="Exh+E",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=False,
        #     id=False,
        #     conflict_avoidance=False,
        #     enumerative=True,
        #     debug=False,
        #     sort_matchings=False,
        #     budget_search=False,
        # ),
        SolverConfig(
            name="Exh+E+B",
            combs=3,
            prune=True,
            enhanced=True,
            pruned_child_gen=False,
            id=False,
            conflict_avoidance=False,
            enumerative=True,
            debug=False,
            sort_matchings=False,
            budget_search=True,
        ),
        SolverConfig(
            name="Exh+E+B+O",
            combs=3,
            prune=True,
            enhanced=True,
            pruned_child_gen=False,
            id=False,
            conflict_avoidance=False,
            enumerative=True,
            debug=False,
            sort_matchings=True,
            budget_search=True,
        ),
        # SolverConfig(
        #     name="Exh+E+B+ID",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=False,
        #     id=True,
        #     conflict_avoidance=True,
        #     enumerative=True,
        #     debug=False,
        #     sort_matchings=False,
        #     budget_search=True,
        # ),
        # SolverConfig(
        #     name="Exh+E+B+O+ID",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=False,
        #     id=True,
        #     conflict_avoidance=True,
        #     enumerative=True,
        #     debug=False,
        #     sort_matchings=True,
        #     budget_search=True,
        # ),
        # SolverConfig(
        #     name="(ictsm)-(enhanced)-(no-id)",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=False,
        #     id=False,
        #     conflict_avoidance=True,
        #     enumerative=False,
        #     debug=False,
        #     sort_matchings=False,
        # ),
        # SolverConfig(
        #     name="(ictsm)-(enhanced)-(id)",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=False,
        #     id=True,
        #     conflict_avoidance=True,
        #     enumerative=False,
        #     debug=False,
        #     sort_matchings=False,
        # ),
        # SolverConfig(
        #     name="test",
        #     combs=3,
        #     prune=True,
        #     enhanced=True,
        #     pruned_child_gen=True,
        #     id=False,
        #     conflict_avoidance=True,
        #     enumerative=True,
        #     debug=False,
        #     sort_matchings=True,
        # ),
    ]
    compare_configs(configs)
