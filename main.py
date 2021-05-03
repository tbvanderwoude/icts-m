import itertools
from itertools import combinations
from copy import copy
import sys

# from matplotlib import pyplot as plt
from pprint import pprint
from collections import deque, defaultdict
from typing import List, Optional, Tuple, Set, DefaultDict, Any, Union, Deque, Iterable

import requests
from mapfmclient import MapfBenchmarker, Problem, Solution, MarkedLocation

# from graphviz import Digraph
from astar import astar
from compact_location import CompactLocation, expand_location, compact_location
from maze import Maze
from mdd import MDD

depth_factor = 1000000

# type alias for graph structure of MDD

JointSolution = List[Tuple[Tuple[CompactLocation, ...], int]]



def is_goal_state(mdds: List[MDD], curr_nodes: List[CompactLocation], curr_depth: int) -> bool:
    for mdd, node in zip(mdds, curr_nodes):
        if curr_depth < mdd.depth or mdd.goal != node:
            return False
    return True


"""Generates locations to choose for each (MDD,Location) pair."""


def get_children_for_mdds(mdds: List[MDD], curr_nodes: List[CompactLocation], curr_depth: int) -> Iterable[
    List[CompactLocation]]:
    return map(lambda x: x[0].get_children_at_node(x[1], curr_depth), zip(mdds, curr_nodes))


def is_invalid_move(curr_locs, next_locs):
    return not all_different(curr_locs) or has_edge_collisions(curr_locs, next_locs)


def has_edge_collisions(curr_nodes: List[CompactLocation], next_nodes: List[CompactLocation]) -> bool:
    forward_edges = set(filter(lambda p: p[0] != p[1], zip(curr_nodes, next_nodes)))
    backward_edges = set(filter(lambda p: p[0] != p[1], zip(next_nodes, curr_nodes)))
    return not forward_edges.isdisjoint(backward_edges)


def prune_joint_children(joint_child_nodes, curr_nodes: List[CompactLocation]):
    return filter(
            lambda node: all_different(node) and not has_edge_collisions(curr_nodes, node), joint_child_nodes)


def get_valid_children(mdds: List[MDD], curr_nodes: List[CompactLocation], curr_depth: int, unfold_mdds: bool = False, accumulator: List = []):
    per_mdd_children = get_children_for_mdds(mdds, curr_nodes, curr_depth)
    all_joint_child_nodes = itertools.product(*per_mdd_children)
    pruned = prune_joint_children(all_joint_child_nodes, curr_nodes)
    if unfold_mdds:
        for (i, children) in enumerate(per_mdd_children):
            node = curr_nodes[i]
            for child in children:
                occurs = False
                for pruned_joint in pruned:
                    if pruned_joint[i] == child:
                        occurs = True
                        break
                if not occurs:
                    if curr_depth < mdds[i].depth:
                        mdds[i].mdd[(node, curr_depth)].remove((child, curr_depth + 1))
                        accumulator.append((i,(node, curr_depth),(child, curr_depth + 1)))

    return pruned


# def get_children_for_mdds(mdds: List[MDD], curr_nodes: List[Location], curr_depth: int) -> List[List[Location]]:
#     return list(map(lambda x: x[0].get_children_at_node(x[1], curr_depth), zip(mdds, curr_nodes)))
#
# def get_children_at_node(self, node: Location, curr_depth: int) -> List[Location]:
#     if self.goal == node and curr_depth >= self.depth:
#         return [self.goal]
#     else:
#         return list(map(lambda p: p[0], self.mdd[(node, curr_depth)]))


def seek_solution_in_joint_mdd(mdds: List[MDD], constructive: bool, unfold: bool = False, accumulator: List = []) -> Union[bool, JointSolution]:
    for mdd in mdds:
        if not mdd.mdd:
            if constructive:
                return []
            else:
                return False
    roots: Tuple[Any] = tuple(map(lambda mdd: mdd.start, mdds))
    depths: Iterable[int] = map(lambda mdd: mdd.depth, mdds)
    visited = set()
    if constructive:
        solution, _ = joint_mdd_dfs_constructive(mdds, None, (roots, 0), max(depths), visited)
        return solution
    else:
        found_path, visited = joint_mdd_dfs(mdds, (roots, 0), max(depths), visited, unfold, accumulator)
        return found_path


def joint_mdd_dfs(mdds: List[MDD], curr: Tuple[Any, int], max_depth: int,
                  visited: Set[Tuple[List[CompactLocation], int]], unfold: bool = False,  accumulator: List = []) \
        -> Tuple[bool, Set[Tuple[List[CompactLocation], int]]]:
    curr_nodes: List[CompactLocation] = curr[0]
    curr_depth: int = curr[1]
    if curr in visited or curr_depth > max_depth:
        return False, visited
    visited.add(curr)
    if is_goal_state(mdds, curr_nodes, curr_depth):
        return True, visited
    children = get_valid_children(mdds, curr_nodes, curr_depth, unfold, accumulator)
    for node in children:
        child = (node, curr_depth + 1)
        found_path, visited = joint_mdd_dfs(mdds, child, max_depth, visited,unfold, accumulator)
        if found_path:
            return found_path, visited
    return False, visited


def joint_mdd_dfs_constructive(mdds: List[MDD], prev: Optional[List[CompactLocation]], curr: Tuple[Any, int],
                               max_depth: int,
                               visited: Set[Tuple[List[CompactLocation], int]]) \
        -> Tuple[JointSolution, Set[Tuple[List[CompactLocation], int]]]:
    curr_nodes: List[CompactLocation] = curr[0]
    curr_depth: int = curr[1]
    if prev and is_invalid_move(prev, curr_nodes):
        return [], visited
    if curr in visited or curr_depth > max_depth:
        return [], visited

    visited.add(curr)
    if is_goal_state(mdds, curr_nodes, curr_depth):
        return [curr], visited
    children = get_valid_children(mdds, curr_nodes, curr_depth)

    partial_sol = [curr]
    for node in children:
        child = (node, curr_depth + 1)
        if child not in visited:
            sol, visited = joint_mdd_dfs_constructive(mdds, curr_nodes, child, max_depth, visited)
            if sol:
                partial_sol.extend(sol)
                return partial_sol, visited
    return [], visited

# early-exit variant of the version below. Intuitively, it should be faster
# https://stackoverflow.com/questions/5278122/checking-if-all-elements-in-a-list-are-unique
# experimentally, it is much much worse!
# def all_different(xs):
#     seen = set()
#     return not any(i in seen or seen.add(i) for i in xs)

def all_different(xs):
    return len(set(xs)) == len(xs)


def find_number_of_open_spaces(maze: Maze):
    return sum([sum([x ^ 1 for x in row]) for row in maze.grid])


def calculate_upper_bound_cost(agents: int, maze: Maze):
    number_of_open_spaces = find_number_of_open_spaces(maze)
    return (agents ** 2) * number_of_open_spaces


def ict_search(maze: Maze, combs, prune, enhanced, subproblems: List[Tuple[CompactLocation, CompactLocation]], root: Tuple[int, ...]) -> \
Optional[
    JointSolution]:
    frontier = deque()
    frontier.append(root)
    visited = set()
    mdd_cache = dict()

    k = len(subproblems)
    upper = calculate_upper_bound_cost(k, maze)
    print(k, upper, root)
    while frontier:
        node = frontier.popleft()
        if sum(node) <= upper and not node in visited:
            accumulator = []
            visited.add(node)
            mdds = []
            for (i, c) in enumerate(node):
                node_list = list(node)
                node_list[i] += 1
                if not tuple(node_list) in visited:
                    frontier.append(tuple(node_list))
                if not (i, c) in mdd_cache:
                    if (i, c - 1) in mdd_cache:
                        mdd_cache[(i, c)] = MDD(maze, i, subproblems[i][0], subproblems[i][1], c, mdd_cache[(i, c - 1)])
                    else:
                        mdd_cache[(i, c)] = MDD(maze, i, subproblems[i][0], subproblems[i][1], c)
                mdds.append(mdd_cache[(i, c)])
            if not prune or k <= combs or check_combinations(combs, mdds, k, enhanced,accumulator):
                solution: JointSolution = seek_solution_in_joint_mdd(mdds, True)
                if solution:
                    return solution
            for (i, p, c) in accumulator:
                mdds[i].mdd[p].add(c)
    return []


def check_combinations(combs, mdds: List[MDD], k: int, enhanced: bool = True, accumulator: List = []):
    for c in combinations(range(k), combs):
        if not seek_solution_in_joint_mdd([mdds[i] for i in c], False, enhanced, accumulator):
            return False
    return True


def enumerate_matchings(agents, tasks):
    if agents:
        (name, type), *tail = agents
        results = []
        for (i, (task_name, task_type)) in enumerate(tasks):
            if type == task_type:
                tasks_cp = copy(tasks)
                tasks_cp.pop(i)
                if tail:
                    results.extend(map(lambda rs: [(name, task_name)] + rs, enumerate_matchings(tail, tasks_cp)))
                else:
                    results.append([(name, task_name)])
        return results
    else:
        return []


def solve(problem: Problem) -> Solution:
    combs = 2
    prune = True
    enhanced = True
    maze: Maze = Maze(problem.grid, problem.width, problem.height)
    paths: List[List[Tuple[int, int]]] = []
    agents = list(map(lambda marked: (compact_location(marked.x, marked.y), marked.color), problem.starts))
    goals = list(map(lambda marked: (compact_location(marked.x, marked.y), marked.color), problem.goals))
    matchings = enumerate_matchings(agents, goals)
    min_sic = None
    min_sol = None
    for matching in matchings:
        subproblems = []
        root_list = []
        for (start, goal) in matching:
            subproblems.append((start, goal))
            shortest = astar(maze, start, goal)
            assert len(shortest) > 0
            root_list.append(len(shortest) - 1)
        root = tuple(root_list)
        sol = ict_search(maze, combs,prune,enhanced,subproblems, root)
        stripped_sol = list(map(lambda x: x[0], sol))
        sic = len(stripped_sol)
        if not min_sic or min_sic > sic:
            min_sic = sic
            min_sol = stripped_sol
    # print(min_sic)
    subsols = list(zip(*min_sol))
    for subsol in subsols:
        paths.append(list(map(lambda loc: expand_location(loc), subsol)))
    return Solution.from_paths(paths)


def run_custom(token,p_id):
    headers = {
        'X-API-Token': token
    }
    data = {
        "algorithm": "ICTS",
        "version": "0.1.2",
        "debug": True
    }
    r = requests.post("https://mapf.nl/api/benchmark/{}".format(p_id), headers=headers,
                      json=data)
    assert r.status_code == 200, print(r.content)
    j = r.json()
    problem_json = j["problems"][0]
    problem = Problem(problem_json["grid"], problem_json["width"], problem_json["height"], [MarkedLocation.from_dict(i) for i in problem_json["starts"]],
                           [MarkedLocation.from_dict(i) for i in problem_json["goals"]], 0, problem_json["id"], 0)
    solution = solve(problem)
    pprint(solution.serialize())

def str_to_bool(s):
    return s == "true" or s == "True"
if __name__ == '__main__':
    token = "FXJ8wNVeWh4syRdh"
    p_id = int(sys.argv[1])
    profile = str_to_bool(sys.argv[2])
    debug = str_to_bool(sys.argv[3])
    # print(p_id,profile)
    if profile:
        run_custom(token,p_id)
    else:
        benchmark = MapfBenchmarker(
            token=token, problem_id=p_id,
            algorithm="ICTS", version="0.1.3",
            debug=debug, solver=solve,
            cores=8
        )
        benchmark.run()
