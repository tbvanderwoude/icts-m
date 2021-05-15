from mapfmclient import Problem, MarkedLocation
import random

from mapfm.solve import solve


def p_bool(p):
    return random.random() < p


def gen_rand_grid(width, height, p):
    rows = []
    for y in range(height):
        rows.append([int(p_bool(p)) for x in range(width)])
    return rows


def gen_legal_point(grid, width, height):
    point = None
    while not point:
        start_x = random.randrange(width)
        start_y = random.randrange(height)
        if not grid[start_y][start_x]:
            point = start_x, start_y
    return point


def gen_agent_goals(grid, width, height, k):
    num_teams = random.randrange(1, k)
    starts = []
    goals = []
    for agent in range(k):
        team = random.randrange(num_teams)
        start_x, start_y = gen_legal_point(grid, width, height)
        goal_x, goal_y = gen_legal_point(grid, width, height)
        start = MarkedLocation(team, start_x, start_y)
        goal = MarkedLocation(team, goal_x, goal_y)
        starts.append(start)
        goals.append(goal)
    return starts, goals


if __name__ == "__main__":
    width = 16
    height = 16
    grid = gen_rand_grid(width, height, 0.1)
    starts, goals = gen_agent_goals(grid, width, height, 3)
    problem = Problem(grid, width, height, starts, goals)
    solution = solve(problem)
    print(grid)
