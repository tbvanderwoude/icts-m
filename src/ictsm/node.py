from typing import Optional

from .compact_location import CompactLocation


class Node(object):
    __slots__ = ["parent", "loc", "cost", "heuristic"]

    def __init__(self, parent, loc: CompactLocation, cost: int, heuristic: int):
        self.parent: Optional[Node] = parent
        self.loc: CompactLocation = loc
        self.cost: int = cost
        self.heuristic: int = heuristic

    def __hash__(self):
        return hash(self.loc)

    def is_root(self):
        return self.parent is None

    def is_goal(self, goal: CompactLocation) -> bool:
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
