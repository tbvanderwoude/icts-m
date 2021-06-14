from collections import defaultdict
from typing import List, Optional, Tuple, Iterable, TypeVar, Sized, DefaultDict, Set

from .compact_location import CompactLocation

# early-exit variant of the version below. Intuitively, it should be faster
# https://stackoverflow.com/questions/5278122/checking-if-all-elements-in-a-list-are-unique
# experimentally, it is much much worse!
# def all_different(xs):
#     seen = set()
#     return not any(i in seen or seen.add(i) for i in xs)

Conflict = Tuple[int, int]
a = TypeVar("a")


def all_different(xs: List[a]):
    return len(set(xs)) == len(xs)


def is_invalid_move(curr_locs, next_locs):
    return not all_different(curr_locs) or has_edge_collisions(curr_locs, next_locs)


def all_different_constructive(xs: Iterable[CompactLocation]) -> Optional[Conflict]:
    loc_agent: DefaultDict[CompactLocation, Set[int]] = defaultdict(set)
    for (i, curr_loc) in enumerate(xs):
        if len(loc_agent[curr_loc]) > 0:
            return list(loc_agent[curr_loc])[0], i
        else:
            loc_agent[curr_loc].add(i)
    return None


def count_all_different(xs: Iterable[CompactLocation]) -> int:
    count = 0
    loc_agent: DefaultDict[CompactLocation, Set[int]] = defaultdict(set)
    for (i, curr_loc) in enumerate(xs):
        if len(loc_agent[curr_loc]) > 0:
            count += 1
        else:
            loc_agent[curr_loc].add(i)
    return count


def count_edge_collsions(curr_locs, next_locs) -> int:
    count = 0
    for (i, ei) in filter(
        lambda p: p[1][0] != p[1][1], enumerate(zip(curr_locs, next_locs))
    ):
        for (j, ej) in filter(
            lambda p: p[1][0] != p[1][1], enumerate(zip(next_locs, curr_locs))
        ):
            if j > i and ei == ej:
                count += 1
    return count


def count_conflicts(curr_locs, next_locs) -> int:
    return count_all_different(curr_locs) + count_edge_collsions(curr_locs, next_locs)


def edge_collisions_constructive(curr_locs, next_locs) -> Optional[Conflict]:
    for (i, ei) in filter(
        lambda p: p[1][0] != p[1][1], enumerate(zip(curr_locs, next_locs))
    ):
        for (j, ej) in filter(
            lambda p: p[1][0] != p[1][1], enumerate(zip(next_locs, curr_locs))
        ):
            if ei == ej:
                return i, j
    return None

def find_conflict(curr_locs, next_locs) -> Optional[Conflict]:
    all_diff = all_different_constructive(curr_locs)
    if all_diff:
        return all_diff
    edge_colls = edge_collisions_constructive(curr_locs, next_locs)
    if edge_colls:
        return edge_colls
    return None


def has_edge_collisions(
    curr_nodes: List[CompactLocation], next_nodes: List[CompactLocation]
) -> bool:
    forward_edges = set(filter(lambda p: p[0] != p[1], zip(curr_nodes, next_nodes)))
    backward_edges = set(filter(lambda p: p[0] != p[1], zip(next_nodes, curr_nodes)))
    return not forward_edges.isdisjoint(backward_edges)
