import heapq
from typing import List, Optional, Tuple
from mapfmclient import MapfBenchmarker, Problem, Solution, MarkedLocation


class Location:
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y

    def __eq__(self, other) -> bool:
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x,self.y))

    @classmethod
    def from_dict(cls, dct) -> "Location":
        return cls(dct["x"], dct["y"])


class Node:
    def __init__(self, parent, loc: Location, cost: int, heuristic: int):
        self.parent: Optional[Node] = parent
        self.loc: Location = loc
        self.cost: int = cost
        self.heuristic: int = heuristic

    def __key(self):
        return (self.attr_a, self.attr_b, self.attr_c)

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
        all_children = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1), (x, y)]
        good_children = []
        for c in all_children:
            if c[0] >= 0 and c[0] < self.height and c[1] >= 0 and c[1] < self.width:
                if not self.grid[c[1]][c[0]]:
                    good_children.append(c)
        return list(map(lambda x: Location(x[0], x[1]), good_children))


def heuristic(node: Location, goal: Location) -> int:
    return abs(goal.x - node.x) + abs(goal.y - node.y)


def astar(maze: Maze, start: Location, goal: Location) -> List[Tuple[int,int]]:
    ls: List[Node] = [Node(None, start, 0, heuristic(start, goal))]
    heapq.heapify(ls)
    seen = set()
    while ls:
        n: Node = heapq.heappop(ls)
        if (n.loc) not in seen:
            seen.add(n.loc)
            if n.is_goal(goal):
                return list(map(lambda loc: (loc.x,loc.y),n.get_directions()))
            for c in maze.get_valid_children(n.loc):
                heapq.heappush(ls, Node(n, c, n.cost + 1, heuristic(c, goal)))
    return []


def solve(problem: Problem) -> Solution:
    maze = Maze(problem.grid, problem.width, problem.height)
    paths = []

    assert len(problem.starts) == 1
    assert len(problem.goals) == 1

    start = problem.starts[0]
    goal = problem.goals[0]
    path = astar(maze,Location(start.x,start.y),Location(goal.x,goal.y))
    print("Length of found solution: {}".format(len(path)))
    paths.append(path)
    """
    Now paths looks like:

    paths = list[Path]
    Path = List[(x, y)]
    """
    return Solution.from_paths(paths)


if __name__ == '__main__':
    # start: MarkedLocation = MarkedLocation(0,1,1)
    # goal: MarkedLocation = MarkedLocation(0,2,1)
    # problem: Problem = Problem([
    #     [1,1,1,1],
    #     [1,0,0,1],
    #     [1,1,1,1]
    # ],4,3,[start],[goal],0,1,1)
    # solution = solve(problem)
    # print(solution.serialize())
    benchmark = MapfBenchmarker(
        token="FXJ8wNVeWh4syRdh", problem_id=2,
        algorithm="A*", version="test",
        debug=False, solver=solve,
        cores=8
    )
    benchmark.run()
