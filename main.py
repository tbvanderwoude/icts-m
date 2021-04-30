import heapq
import itertools
from itertools import combinations
from copy import copy, deepcopy
import sys

# from matplotlib import pyplot as plt
from pprint import pprint
from collections import deque, defaultdict
from typing import List, Optional, Tuple, Set, DefaultDict, Any, Union, Deque

import requests
from mapfmclient import MapfBenchmarker, Problem, Solution, MarkedLocation
# from graphviz import Digraph

row_factor = 1000
depth_factor = 1000000


class Location:
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y

    def __eq__(self, other) -> bool:
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return self.x + self.y * row_factor

    def __str__(self):
        return '(' + str(self.x) + ',' + str(self.y) + ')'

    @classmethod
    def from_dict(cls, dct) -> "Location":
        return cls(dct["x"], dct["y"])


class Node:
    def __init__(self, parent, loc: Location, cost: int, heuristic: int):
        self.parent: Optional[Node] = parent
        self.loc: Location = loc
        self.cost: int = cost
        self.heuristic: int = heuristic

    def __hash__(self):
        return hash(self.loc)

    def is_root(self):
        return self.parent is None

    def is_goal(self, goal: Location) -> bool:
        return self.loc == goal

    def get_directions(self):
        if self.is_root():
            return [self.loc]
        else:
            par_dirs = self.parent.get_directions()
            par_dirs.append(self.loc)
            return par_dirs

    def __eq__(self, other) -> bool:
        return self.cost + self.heuristic == other.cost + other.heuristic

    def __ne__(self, other) -> bool:
        return self.cost + self.heuristic != other.cost + other.heuristic

    def __ge__(self, other) -> bool:
        return self.cost + self.heuristic >= other.cost + other.heuristic

    def __lt__(self, other) -> bool:
        return self.cost + self.heuristic < other.cost + other.heuristic

    def __gt__(self, other) -> bool:
        return self.cost + self.heuristic > other.cost + other.heuristic

    def __le__(self, other) -> bool:
        return self.cost + self.heuristic <= other.cost + other.heuristic


class Maze:
    def __init__(self, grid: List[List[int]], width: int, height: int):
        self.grid = grid
        self.width = width
        self.height = height

    def get_valid_children(self, loc: Location) -> List[Location]:
        x, y = loc.x, loc.y
        all_children: List[Tuple[int, int]] = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1), (x, y)]
        good_children: List[Tuple[int, int]] = []
        for c in all_children:
            if 0 <= c[0] < self.width and 0 <= c[1] < self.height:
                if not self.grid[c[1]][c[0]]:
                    good_children.append(c)
        return list(map(lambda x: Location(x[0], x[1]), good_children))


def heuristic(node: Location, goal: Location) -> int:
    return abs(goal.x - node.x) + abs(goal.y - node.y)


def astar(maze: Maze, start: Location, goal: Location) -> List[Tuple[int, int]]:
    ls: List[Node] = [Node(None, start, 0, heuristic(start, goal))]
    heapq.heapify(ls)
    seen: Set[Location] = set()
    while ls:
        n: Node = heapq.heappop(ls)
        if (n.loc) not in seen:
            seen.add(n.loc)
            if n.is_goal(goal):
                return list(map(lambda loc: (loc.x, loc.y), n.get_directions()))
            for c in maze.get_valid_children(n.loc):
                heapq.heappush(ls, Node(n, c, n.cost + 1, heuristic(c, goal)))
    return []


# type alias for graph structure of MDD
MDDGraph = Optional[DefaultDict[Tuple[Location, int], Set[Tuple[Location, int]]]]

JointSolution = List[Tuple[Tuple[Location, ...], int]]


class MDD:
    def __init__(self, maze: Maze, agent: int, start: Location, goal: Location, depth: int, last_mdd=None):
        self.agent: int = agent
        self.start: Location = start
        self.goal: Location = goal
        self.depth: int = depth
        self.bfs_tree = {}
        self.mdd: MDDGraph = None
        self.level: DefaultDict[int, Set[Location]] = defaultdict(set)
        if last_mdd and last_mdd.depth < depth and last_mdd.agent == agent:
            self.generate_mdd(maze, last_mdd)
        else:
            self.generate_mdd(maze)

    def generate_mdd(self, maze, last_mdd=None):
        if last_mdd:
            bfs_tree = bootstrap_depth_d_bfs_tree(maze, self.depth, last_mdd.bfs_tree)
        else:
            bfs_tree = construct_bfs_tree(maze, self.start, self.depth)
        self.bfs_tree = bfs_tree
        mdd = mdd_from_tree(self.bfs_tree['tree'], self.goal, self.depth)
        self.mdd = mdd
        if mdd:
            self.populate_levels(self.mdd)

    # Constructs a graph of the MDD.
    # def show(self):
    #     items = list(sorted(self.mdd.items(), key=lambda x: x[0][1]))
    #     g = Digraph()
    #     added = set()
    #     plt.tight_layout()
    #     for ((loc, d), v) in items:
    #         node_str = str(loc) + ',' + str(d)
    #         g.node(node_str)
    #         for (c_loc, c_depth) in v:
    #             child_str = str(c_loc) + ',' + str(c_depth)
    #             if not child_str in added:
    #                 added.add(child_str)
    #             g.edge(node_str, child_str)
    #     return g

    def populate_levels(self, mdd: MDDGraph):
        # all nodes except the start are children of other nodes at a level given by the depth
        self.level[0] = {self.start}
        for children_sets in mdd.values():
            for child in children_sets:
                self.level[child[1]].add(child[0])

    def get_level(self, i):
        # Models the behaviour of staying at the goal once reached
        if i > self.depth:
            return {self.goal}
        return self.level[i]

    def get_children_at_node(self, node: Location, curr_depth: int) -> List[Location]:
        if self.goal == node and curr_depth >= self.depth:
            return [self.goal]
        else:
            return list(map(lambda p: p[0], self.mdd[(node, curr_depth)]))


"""
Constructs a top-down MDD structure in the form of a dictionary of parent-children mappings representing all the ways to
get to the goal-node. This is done by tracing paths back to the start through the bottom-up DAG that was constructed
when doing the breadth-first traversal. This is similar to tracing a single path back up as in normal path-finding (consider
the get_directions function in the A* Node class), but here there are typically many paths that can be taken, resulting 
in an MDD.
TLDR: turns a child-parents structure into a parent-children structure with some filtering along the way
"""


def mdd_from_tree(tree: DefaultDict[Tuple[Location, int], Set[Tuple[Location, int]]], goal: Location, depth: int) \
        -> MDDGraph:
    goal_at_depth = (goal, depth)
    # If the goal node is not in the DAG, return the empty MDD represented by None
    if not tree[goal_at_depth]:
        return None
    visited = set()
    mdd = defaultdict(set)
    trace_list = deque()
    for parent in tree[goal_at_depth]:
        trace_list.append((parent, goal_at_depth))
        visited.add((parent, goal_at_depth))
    while trace_list:
        current, child = trace_list.popleft()
        mdd[current].add(child)
        for parent in tree[current]:
            if (parent, current) not in visited:
                trace_list.append((parent, current))
                visited.add((parent, current))
    return mdd


def construct_bfs_tree(maze: Maze, start: Location, depth: int):
    fringe = deque()
    fringe.append((start, 0))
    # DAG represented by child-parents map. This formulation makes it easier to construct the path(s) from the parents
    # to the child later, similar to the get_directions function in the A* Node class
    prev_dict = defaultdict(set)
    visited = set()
    return main_bfs_loop(maze, depth, fringe, prev_dict, visited)


def bootstrap_depth_d_bfs_tree(maze: Maze, depth: int, old_tree):
    fringe = deque()
    old_fringe = list(old_tree['fringe'])
    old_fringe.sort(key=lambda x: x[0].x + x[0].y)
    fringe.extend(old_fringe)
    prev_dict = old_tree['tree']
    for node in old_fringe:
        node_prevs = old_tree['fringe_prevs'][node]
        prev_dict[node].update(node_prevs)
    visited = old_tree['visited']
    new_bfs_tree = main_bfs_loop(maze, depth, fringe, prev_dict, visited)
    return new_bfs_tree


def main_bfs_loop(maze: Maze, depth: int, fringe: Deque[Tuple[Location, int]], prev_dict, visited):
    depth_d_plus_one_fringe = set()
    fringe_prevs = defaultdict(set)
    while fringe:
        curr = fringe.popleft()
        loc, d = curr
        children: List[Tuple[Location, int]] = list(map(lambda c: (c, d + 1), maze.get_valid_children(loc)))
        for c in children:
            if c[1] <= depth:
                prev_dict[c].add(curr)
                if not c in visited:
                    fringe.append(c)
                    visited.add(c)
            if c[1] == depth + 1:
                depth_d_plus_one_fringe.add(c)
                fringe_prevs[c].add(curr)
    return {'tree': prev_dict, 'visited': visited, 'depth': depth, 'fringe': depth_d_plus_one_fringe,
            'fringe_prevs': fringe_prevs}


def is_goal_state(mdds: List[MDD], curr_nodes: List[Location], curr_depth: int) -> bool:
    for mdd, node in zip(mdds, curr_nodes):
        if curr_depth < mdd.depth or mdd.goal != node:
            return False
    return True


"""Generates locations to choose for each (MDD,Location) pair."""


def get_children_for_mdds(mdds: List[MDD], curr_nodes: List[Location], curr_depth: int) -> List[List[Location]]:
    return list(map(lambda x: x[0].get_children_at_node(x[1], curr_depth), zip(mdds, curr_nodes)))


def is_invalid_move(curr_locs, next_locs):
    return not all_different(curr_locs) or has_edge_collisions(curr_locs, next_locs)


def has_edge_collisions(curr_nodes: List[Location], next_nodes: List[Location]) -> bool:
    forward_edges = set(filter(lambda p: p[0] != p[1], zip(curr_nodes, next_nodes)))
    backward_edges = set(filter(lambda p: p[0] != p[1], zip(next_nodes, curr_nodes)))
    return not forward_edges.isdisjoint(backward_edges)


def prune_joint_children(joint_child_nodes, curr_nodes: List[Location]):
    return list(
        filter(
            lambda node: all_different(node) and not has_edge_collisions(curr_nodes, node), joint_child_nodes))

def get_valid_children(mdds: List[MDD], curr_nodes: List[Location], curr_depth: int, unfold_mdds: bool = False):
    per_mdd_children = get_children_for_mdds(mdds, curr_nodes, curr_depth)
    all_joint_child_nodes = list(itertools.product(*per_mdd_children))
    pruned = prune_joint_children(all_joint_child_nodes, curr_nodes)
    if unfold_mdds:
        for (i,children) in enumerate(per_mdd_children):
            node = curr_nodes[i]
            for child in children:
                occurs = False
                for pruned_joint in pruned:
                    if pruned_joint[i] == child:
                        occurs = True
                        break
                if not occurs:
                    if curr_depth < mdds[i].depth:
                        mdds[i].mdd[(node,curr_depth)].remove((child,curr_depth + 1))

    return pruned

# def get_children_for_mdds(mdds: List[MDD], curr_nodes: List[Location], curr_depth: int) -> List[List[Location]]:
#     return list(map(lambda x: x[0].get_children_at_node(x[1], curr_depth), zip(mdds, curr_nodes)))
#
# def get_children_at_node(self, node: Location, curr_depth: int) -> List[Location]:
#     if self.goal == node and curr_depth >= self.depth:
#         return [self.goal]
#     else:
#         return list(map(lambda p: p[0], self.mdd[(node, curr_depth)]))


def seek_solution_in_joint_mdd(mdds: List[MDD], constructive: bool, unfold: bool = False) -> Union[bool, JointSolution]:
    for mdd in mdds:
        if not mdd.mdd:
            if constructive:
                return []
            else:
                return False
    roots: Tuple[Any] = tuple(map(lambda mdd: mdd.start, mdds))
    depths: List[int] = list(map(lambda mdd: mdd.depth, mdds))
    visited = set()
    if constructive:
        solution, _ = joint_mdd_dfs_constructive(mdds, None, (roots, 0), max(depths), visited)
        return solution
    else:
        found_path, visited = joint_mdd_dfs(mdds, (roots, 0), max(depths), visited,unfold)
        return found_path


def joint_mdd_dfs(mdds: List[MDD], curr: Tuple[Any, int], max_depth: int,
                  visited: Set[Tuple[List[Location], int]], unfold: bool = False) \
        -> Tuple[bool, Set[Tuple[List[Location], int]]]:
    curr_nodes: List[Location] = curr[0]
    curr_depth: int = curr[1]
    if curr in visited or curr_depth > max_depth:
        return False, visited
    visited.add(curr)
    if is_goal_state(mdds, curr_nodes, curr_depth):
        return True, visited
    children = get_valid_children(mdds, curr_nodes, curr_depth, unfold)
    for node in children:
        child = (node, curr_depth + 1)
        found_path, visited = joint_mdd_dfs(mdds, child, max_depth, visited)
        if found_path:
            return found_path, visited
    return False, visited


def joint_mdd_dfs_constructive(mdds: List[MDD], prev: Optional[List[Location]], curr: Tuple[Any, int], max_depth: int,
                               visited: Set[Tuple[List[Location], int]]) \
        -> Tuple[JointSolution, Set[Tuple[List[Location], int]]]:
    curr_nodes: List[Location] = curr[0]
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


def all_different(xs):
    return len(set(xs)) == len(xs)


def find_number_of_open_spaces(maze: Maze):
    return sum([sum([x^1 for x in row]) for row in maze.grid])


def calculate_upper_bound_cost(agents: int, maze: Maze):
    number_of_open_spaces = find_number_of_open_spaces(maze)
    return (agents ** 2) * number_of_open_spaces


def ict_search(maze: Maze, subproblems: List[Tuple[Location, Location]], root: Tuple[int, ...]) -> Optional[
    JointSolution]:
    frontier = deque()
    frontier.append(root)
    visited = set()
    mdd_cache = dict()
    k = len(subproblems)
    upper = calculate_upper_bound_cost(k, maze)
    print(k,upper,root)
    while frontier:
        node = frontier.popleft()
        if sum(node) <= upper and not node in visited:
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
                mdds.append(copy(mdd_cache[(i, c)]))
                mdds[i].mdd = deepcopy(mdd_cache[(i, c)].mdd)
            if k <= 2 or check_triples(mdds,k):
                solution: JointSolution = seek_solution_in_joint_mdd(mdds, True)
                if solution:
                    return solution
    return []

def check_triples(mdds: List[MDD],k: int, enhanced: bool = True):
    for (i,j) in combinations(range(k), 2):
        if not seek_solution_in_joint_mdd([mdds[i],mdds[j]], False,enhanced):
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
                    results.extend(list(map(lambda rs: [(name, task_name)] + rs, enumerate_matchings(tail, tasks_cp))))
                else:
                    results.append([(name, task_name)])
        return results
    else:
        return []


def solve(problem: Problem) -> Solution:
    maze: Maze = Maze(problem.grid, problem.width, problem.height)
    paths: List[List[Tuple[int, int]]] = []
    agents = list(map(lambda marked: (Location(marked.x, marked.y), marked.color), problem.starts))
    goals = list(map(lambda marked: (Location(marked.x, marked.y), marked.color), problem.goals))
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
        sol = ict_search(maze, subproblems, root)
        stripped_sol = list(map(lambda x: x[0], sol))
        sic = len(stripped_sol)
        if not min_sic or min_sic > sic:
            min_sic = sic
            min_sol = stripped_sol
    print(min_sic)
    subsols = list(zip(*min_sol))
    for subsol in subsols:
        paths.append(list(map(lambda loc: (loc.x, loc.y), subsol)))
    return Solution.from_paths(paths)


if __name__ == '__main__':
    # start1: MarkedLocation = MarkedLocation(0, 1, 1)
    # start2: MarkedLocation = MarkedLocation(1, 3, 1)
    # goal1: MarkedLocation = MarkedLocation(0, 3, 1)
    # goal2: MarkedLocation = MarkedLocation(1, 3, 3)
    # problem: Problem = Problem([
    #     [1, 1, 1, 1, 1],
    #     [1, 0, 1, 0, 1],
    #     [1, 0, 0, 0, 1],
    #     [1, 0, 1, 0, 1],
    #     [1, 1, 1, 1, 1]
    # ], 5, 5, [start1, start2], [goal1, goal2], 0, 1, 1)
    # solution = solve(problem)
    # pprint(solution.serialize())
    # agents = [("alice",1),("bob",1),("eve",2)]
    # tasks = [("encrypt",1),("decrypt",1),("eavesdrop",2)]
    # print(enumerate_matchings(agents,tasks))
    # print(enumerate_matchings(agents[1:],tasks[1:]))
    # print(enumerate_matchings(agents[2:],tasks[2:]))
    token = "FXJ8wNVeWh4syRdh"
    p_id = int(sys.argv[1])
    # benchmark = MapfBenchmarker(
    #     token=token, problem_id=p_id,
    #     algorithm="ICTS", version="0.1.2",
    #     debug=True, solver=solve,
    #     cores=8
    # )
    # benchmark.run()
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