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


def gen_legal_point(taken,grid, width, height):
    point = None
    while not point:
        start_x = random.randrange(width)
        start_y = random.randrange(height)
        if not grid[start_y][start_x] and not (start_x,start_y) in taken:
            point = start_x, start_y
    return point


def gen_agent_goals(grid, width, height, k):
    num_teams = 1
    starts = []
    goals = []
    taken_starts = set()
    taken_goals = set()
    for agent in range(k):
        team = random.randrange(num_teams)
        start_x, start_y = gen_legal_point(taken_starts,grid, width, height)
        taken_starts.add((start_x,start_y))
        goal_x, goal_y = gen_legal_point(taken_goals,grid, width, height)
        taken_goals.add((goal_x,goal_y))
        start = MarkedLocation(team, start_x, start_y)
        goal = MarkedLocation(team, goal_x, goal_y)
        starts.append(start)
        goals.append(goal)
    return starts, goals

def show_problem(grid, width,height, starts,goals):
    for y in range(height):
        s = ""
        for x in range(width):
            if grid[y][x]:
                s+='#'
            else:
                s+=' '
        print(s)
    return starts, goals

if __name__ == "__main__":
    width = 8
    height = 8
    grid = gen_rand_grid(width, height, 0.1)
    starts, goals = gen_agent_goals(grid, width, height, 2)
    # show_problem(grid,width,height,starts,goals)
    problem = Problem(grid, width, height, starts, goals)
    enum_sol = solve(problem,True)
    tapf_sol = solve(problem,False)
    # print(grid)
