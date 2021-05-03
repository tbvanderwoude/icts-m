ROW_FACTOR = 2048

CompactLocation = int


def compact_location(x, y):
    return x + y * ROW_FACTOR


def expand_location(c: CompactLocation):
    return c % ROW_FACTOR, c // ROW_FACTOR


def manhattan_norm(node: CompactLocation, goal: CompactLocation) -> int:
    x1, y1 = expand_location(node)
    x2, y2 = expand_location(goal)
    return abs(x1 - x2) + abs(y1 - y2)
