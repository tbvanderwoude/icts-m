from typing import List, Tuple

from .compact_location import CompactLocation, expand_location, compact_location


class Maze(object):
    __slots__ = ["grid", "width", "height"]

    def __init__(self, grid: List[List[int]], width: int, height: int):
        self.grid = grid
        self.width = width
        self.height = height

    def get_valid_children(self, loc: CompactLocation) -> List[CompactLocation]:
        x, y = expand_location(loc)
        all_children: List[Tuple[int, int]] = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
            (x, y),
        ]
        good_children: List[CompactLocation] = []
        for c in all_children:
            if 0 <= c[0] < self.width and 0 <= c[1] < self.height:
                if not self.grid[c[1]][c[0]]:
                    good_children.append(compact_location(c[0], c[1]))
        return good_children

    def __str__(self):
        accum = ""
        for y in range(self.height):
            accum += str(self.grid[y])
