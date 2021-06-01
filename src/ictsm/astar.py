import heapq
from typing import List, Set, Tuple

from .compact_location import CompactLocation, manhattan_norm, expand_location
from .maze import Maze
from .node import Node


def astar(
    maze: Maze, start: CompactLocation, goal: CompactLocation
) -> List[Tuple[int, int]]:
    ls: List[Node] = [Node(None, start, 0, manhattan_norm(start, goal))]
    heapq.heapify(ls)
    seen: Set[CompactLocation] = set()
    while ls:
        n: Node = heapq.heappop(ls)
        if (n.loc) not in seen:
            seen.add(n.loc)
            if n.is_goal(goal):
                return list(map(lambda loc: expand_location(loc), n.get_directions()))
            for c in maze.get_valid_children(n.loc):
                heapq.heappush(ls, Node(n, c, n.cost + 1, manhattan_norm(c, goal)))
    return []
