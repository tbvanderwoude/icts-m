import itertools
from typing import List, Tuple, Iterable, Any, Union, Set, Optional

from mapfm.compact_location import CompactLocation
from mapfm.conflicts import (
    is_invalid_move,
    has_edge_collisions,
    all_different,
    count_conflicts,
)
from mapfm.id_context import IDContext
from mapfm.mdd import MDD
from mapfm.util import index_path

JointSolution = List[Tuple[CompactLocation, ...]]
JointTimedSolution = List[Tuple[Tuple[CompactLocation, ...], int]]


def is_goal_state(
    mdds: List[MDD], curr_nodes: List[CompactLocation], curr_depth: int
) -> bool:
    for mdd, node in zip(mdds, curr_nodes):
        if curr_depth < mdd.depth or mdd.goal != node:
            return False
    return True


"""Generates locations to choose for each (MDD,Location) pair."""


def get_children_for_mdds(
    mdds: List[MDD], curr_nodes: List[CompactLocation], curr_depth: int
) -> Iterable[List[CompactLocation]]:
    return map(
        lambda x: x[0].get_children_at_node(x[1], curr_depth), zip(mdds, curr_nodes)
    )


def prune_joint_children(joint_child_nodes, curr_nodes: List[CompactLocation]):
    return list(
        filter(
            lambda node: all_different(node)
            and not has_edge_collisions(curr_nodes, node),
            joint_child_nodes,
        )
    )


def get_valid_children(
    mdds: List[MDD],
    curr_nodes: List[CompactLocation],
    curr_depth: int,
    unfold_mdds: bool = False,
    accumulator: List = [],
):
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
                        accumulator.append(
                            (i, (node, curr_depth), (child, curr_depth + 1))
                        )

    return pruned


def seek_solution_in_joint_mdd(
    mdds: List[MDD],
    constructive: bool,
    unfold: bool = False,
    accumulator: List = [],
    context: Optional[IDContext] = None,
) -> Union[bool, JointTimedSolution]:
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
        solution, _ = joint_mdd_dfs_constructive(
            mdds, None, (roots, 0), max(depths), visited, context
        )
        return solution
    else:
        found_path, visited = joint_mdd_dfs(
            mdds, (roots, 0), max(depths), visited, unfold, accumulator, context
        )
        return found_path


def sample_context_node(context: IDContext, depth: int):
    return list(
        map(
            lambda agent: index_path(context.paths[agent], depth, context.lens[agent]),
            context.other_agents,
        )
    )


def joint_mdd_dfs(
    mdds: List[MDD],
    curr: Tuple[Any, int],
    max_depth: int,
    visited: Set[Tuple[List[CompactLocation], int]],
    unfold: bool = False,
    accumulator: List = [],
    context: Optional[IDContext] = None,
) -> Tuple[bool, Set[Tuple[List[CompactLocation], int]]]:
    curr_nodes: List[CompactLocation] = curr[0]
    curr_depth: int = curr[1]
    if curr in visited or curr_depth > max_depth:
        return False, visited
    visited.add(curr)
    if is_goal_state(mdds, curr_nodes, curr_depth):
        return True, visited
    children = get_valid_children(mdds, curr_nodes, curr_depth, unfold, accumulator)
    if context:
        curr_context = sample_context_node(context, curr_depth)
        next_context = sample_context_node(context, curr_depth + 1)
        children.sort(
            key=lambda child: count_conflicts(
                curr_context + list(curr_nodes), next_context + list(child)
            )
        )
    for node in children:
        child = (node, curr_depth + 1)
        found_path, visited = joint_mdd_dfs(
            mdds, child, max_depth, visited, unfold, accumulator, context
        )
        if found_path:
            return found_path, visited
    return False, visited


def joint_mdd_dfs_constructive(
    mdds: List[MDD],
    prev: Optional[List[CompactLocation]],
    curr: Tuple[Any, int],
    max_depth: int,
    visited: Set[Tuple[List[CompactLocation], int]],
    context: Optional[IDContext] = None,
) -> Tuple[JointTimedSolution, Set[Tuple[List[CompactLocation], int]]]:
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
    if context:
        curr_context = sample_context_node(context, curr_depth)
        next_context = sample_context_node(context, curr_depth + 1)
        children.sort(
            key=lambda child: count_conflicts(
                curr_context + list(curr_nodes), next_context + list(child)
            )
        )
    partial_sol = [curr]
    for node in children:
        child = (node, curr_depth + 1)
        if child not in visited:
            sol, visited = joint_mdd_dfs_constructive(
                mdds, curr_nodes, child, max_depth, visited, context
            )
            if sol:
                partial_sol.extend(sol)
                return partial_sol, visited
    return [], visited
