import math
from collections import defaultdict
from itertools import *

from math import factorial


def permutation(xs, n):
    """
    Return the n'th permutation of xs (counting from 0)
    """
    xs = list(xs)
    len_ = len(xs)
    base = factorial(len_)
    assert n < base, "n is too high ({} >= {})".format(n, base)
    for i in range(len_ - 1):
        base //= len_ - i
        offset = n // base
        if offset:
            # rotate selected value into position
            xs[i + 1 : i + offset + 1], xs[i] = xs[i : i + offset], xs[i + offset]
        n %= base
    return xs


def matching_gen(goal_teams):
    perm_ranges = []
    teams = []
    for team in goal_teams:
        t_list = goal_teams[team]
        teams.append(team)
        perm_ranges.append(range(0, math.factorial(len(t_list))))
    for perm_indices in product(*perm_ranges):
        matching = []
        for (t, i) in zip(teams, perm_indices):
            matching.extend(permutation(goal_teams[t], i))
        yield tuple(matching)


if __name__ == "__main__":
    agents, goals = [
        (1, 1),
        (2, 0),
        (3, 0),
        (4, 0),
        (5, 0),
        (6, 0),
        (7, 0),
        (8, 0),
        (9, 0),
        (10, 0),
    ], [(1, 0), (2, 1), (3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0), (10, 0)]
    agents.sort(key=lambda x: x[1])
    goals.sort(key=lambda x: x[1])
    agent_teams = defaultdict(list)
    goal_teams = defaultdict(list)
    for agent in agents:
        agent_teams[agent[1]].append(agent[0])
    for goal in goals:
        goal_teams[goal[1]].append(goal[0])
    print(goal_teams)
    m_generator = matching_gen(goal_teams)
    for m in islice(m_generator, 0, 10):
        print(m)
