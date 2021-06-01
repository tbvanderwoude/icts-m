from collections import defaultdict, deque
from typing import Optional, DefaultDict, Tuple, Set, Iterable, Deque, Dict

from graphviz import Digraph
from matplotlib import pyplot as plt

from ictsm.compact_location import (
    CompactLocation,
    expand_location,
    CompactLocationDepth,
)
from ictsm.maze import Maze

TimeExpandedGraph = DefaultDict[CompactLocationDepth, Set[CompactLocationDepth]]
MDDGraph = Optional[TimeExpandedGraph]


class MDD(object):
    __slots__ = ["agent", "start", "goals", "depth", "bfs_tree", "mdd", "level"]

    def __init__(
        self,
        maze: Maze,
        agent: int,
        start: CompactLocation,
        goals: Set[CompactLocation],
        depth: int,
        last_mdd=None,
    ):
        self.agent: int = agent
        self.start: CompactLocation = start
        self.goals: Set[CompactLocation] = goals
        self.depth: int = depth
        self.bfs_tree: dict = {}
        self.mdd: MDDGraph = None
        self.level: DefaultDict[int, Set[CompactLocation]] = defaultdict(set)
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
        mdd = mdd_from_tree(self.bfs_tree["tree"], self.goals, self.depth)
        self.mdd = mdd
        if mdd:
            self.populate_levels(self.mdd)

    def populate_levels(self, graph: TimeExpandedGraph):
        # all nodes except the start are children of other nodes at a level given by the depth
        self.level[0] = {self.start}
        for children_sets in graph.values():
            for child in children_sets:
                self.level[child[1]].add(child[0])

    def get_level(self, i):
        # Models the behaviour of staying at the goal once reached
        if i > self.depth:
            return self.goals
        return self.level[i]

    def get_children_at_node(
        self, node: CompactLocation, curr_depth: int
    ) -> Iterable[CompactLocation]:
        if node in self.goals and curr_depth >= self.depth:
            return [node]
        else:
            return map(lambda p: p[0], self.mdd[(node, curr_depth)])

    def show(self):
        print(list(self.mdd.items())[0])
        items = list(sorted(self.mdd.items(), key=lambda x: x[0][1]))
        g = Digraph(format="png")
        added = set()
        plt.tight_layout()
        for ((loc, d), v) in items:
            exp_loc = expand_location(loc)
            node_str = str(exp_loc) + "," + str(d)
            g.node(node_str)
            for (c_loc, c_depth) in v:
                exp_c_loc = expand_location(c_loc)
                child_str = str(exp_c_loc) + "," + str(c_depth)
                if not child_str in added:
                    added.add(child_str)
                g.edge(node_str, child_str)
        return g


"""
Constructs a top-down MDD structure in the form of a dictionary of parent-children mappings representing all the ways to
get to the goal-node. This is done by tracing paths back to the start through the bottom-up DAG that was constructed
when doing the breadth-first traversal. This is similar to tracing a single path back up as in normal path-finding (consider
the get_directions function in the A* Node class), but here there are typically many paths that can be taken, resulting 
in an MDD.
TLDR: turns a child-parents structure into a parent-children structure with some filtering along the way
"""


def mdd_from_tree(
    tree: DefaultDict[CompactLocationDepth, Set[CompactLocationDepth]],
    goals: Set[CompactLocation],
    depth: int,
) -> MDDGraph:
    goals_at_depth = [(goal, depth) for goal in goals]
    # If the goal node is not in the DAG, return the empty MDD represented by None
    reachable_goals = list(
        filter(lambda goal_at_depth: tree[goal_at_depth], goals_at_depth)
    )
    if not reachable_goals:
        return None
    visited = set()
    mdd = defaultdict(set)
    trace_list: Deque[Tuple[CompactLocationDepth, CompactLocationDepth]] = deque()
    for goal in reachable_goals:
        for parent in tree[goal]:
            trace_list.append((parent, goal))
            visited.add((parent, goal))
    while trace_list:
        current, child = trace_list.popleft()
        mdd[current].add(child)
        for parent in tree[current]:
            if (parent, current) not in visited:
                trace_list.append((parent, current))
                visited.add((parent, current))
    return mdd


def construct_bfs_tree(maze: Maze, start: CompactLocation, depth: int):
    fringe: Deque[CompactLocationDepth] = deque()
    fringe.append((start, 0))
    # DAG represented by child-parents map. This formulation makes it easier to construct the path(s) from the parents
    # to the child later, similar to the get_directions function in the A* Node class
    prev_dict: Dict[CompactLocationDepth, Set[CompactLocationDepth]] = defaultdict(set)
    visited: Set[CompactLocationDepth] = set()
    return main_bfs_loop(maze, depth, fringe, prev_dict, visited)


def bootstrap_depth_d_bfs_tree(maze: Maze, depth: int, old_tree: Dict):
    fringe: Deque[CompactLocationDepth] = deque()
    old_fringe = list(old_tree["fringe"])
    old_fringe.sort()
    fringe.extend(old_fringe)
    prev_dict = old_tree["tree"]
    for node in old_fringe:
        node_prevs = old_tree["fringe_prevs"][node]
        prev_dict[node].update(node_prevs)
    visited = old_tree["visited"]
    new_bfs_tree = main_bfs_loop(maze, depth, fringe, prev_dict, visited)
    return new_bfs_tree


def main_bfs_loop(
    maze: Maze,
    depth: int,
    fringe: Deque[CompactLocationDepth],
    prev_dict,
    visited: Set[CompactLocationDepth],
):
    depth_d_plus_one_fringe = set()
    fringe_prevs = defaultdict(set)
    while fringe:
        curr = fringe.popleft()
        loc, d = curr
        children: Iterable[CompactLocationDepth] = map(
            lambda c: (c, d + 1), maze.get_valid_children(loc)
        )
        for c in children:
            if c[1] <= depth:
                prev_dict[c].add(curr)
                if not c in visited:
                    fringe.append(c)
                    visited.add(c)
            if c[1] == depth + 1:
                depth_d_plus_one_fringe.add(c)
                fringe_prevs[c].add(curr)
    return {
        "tree": prev_dict,
        "visited": visited,
        "depth": depth,
        "fringe": depth_d_plus_one_fringe,
        "fringe_prevs": fringe_prevs,
    }
