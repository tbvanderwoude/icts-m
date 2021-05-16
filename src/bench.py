from mapfmclient import Problem, MarkedLocation
import random
import numpy as np
import pickle
from mapfm.solve import solve, solve_enum
from slim_testbench import TestBench


def p_bool(p):
    return random.random() < p


def gen_rand_grid(width, height, p):
    rows = []
    for y in range(height):
        rows.append([int(p_bool(p)) for x in range(width)])
    return rows


def gen_legal_point(taken, grid, width, height):
    point = None
    while not point:
        start_x = random.randrange(width)
        start_y = random.randrange(height)
        if not grid[start_y][start_x] and not (start_x, start_y) in taken:
            point = start_x, start_y
    return point



def gen_agent_goals(grid, width, height, k):
    num_agents = k
    num_teams = random.randrange(1, num_agents+1)
    starts = []
    goals = []
    taken_starts = set()
    taken_goals = set()
    taken = set()
    for agent in range(num_agents):
        team = random.randrange(num_teams)
        if agent < num_teams:
            team = agent
        start_x, start_y = gen_legal_point(taken, grid, width, height)
        taken_starts.add((start_x, start_y))
        taken.add((start_x, start_y))
        goal_x, goal_y = gen_legal_point(taken, grid, width, height)
        taken_goals.add((goal_x, goal_y))
        taken.add((goal_x, goal_y))
        start = MarkedLocation(team, start_x, start_y)
        goal = MarkedLocation(team, goal_x, goal_y)
        starts.append(start)
        goals.append(goal)
    return starts, goals


def show_problem(grid, width, height, starts, goals):
    for y in range(height):
        s = ""
        for x in range(width):
            if grid[y][x]:
                s += "#"
            else:
                s += " "
        print(s)
    return starts, goals

def gen_problem(width,height,density,agents):
    grid = gen_rand_grid(width, height, density)
    starts, goals = gen_agent_goals(grid, width, height, agents)
    return Problem(grid, width, height, starts, goals)

# computes success rate and mean solution time
def process_results(solutions):
    return np.array([int(bool(x[1])) for x in solutions]).mean(),np.array([x[2] for x in solutions]).mean()

def solve_setting(k):
    print("k = {}".format(k))
    problems = [gen_problem(16,16,0.0,k) for i in range(100)]
    bench = TestBench(-1,60000)
    enum_sols = bench.solve_problems(solve_enum,problems)
    print("Done with enumerative solving")
    native_sols = bench.solve_problems(solve,problems)
    print("Done with native solving")
    return enum_sols,native_sols

if __name__ == "__main__":
    results_summary = []
    raw_sols = []
    for k in range(1,17):
        enum_sols,native_sols = solve_setting(k)
        raw_sols.append((enum_sols,native_sols))
        enum_success, enum_mean_time = process_results(enum_sols)
        native_success, native_mean_time = process_results(native_sols)
        print(enum_success, native_success, enum_mean_time, native_mean_time)
        results_summary.append((enum_success, native_success, enum_mean_time, native_mean_time))
    print(results_summary)
    results_summary = np.array(results_summary)
    filename = 'raw_results.txt'
    outfile = open(filename, 'wb')
    pickle.dump(raw_sols, outfile)
    outfile.close()
    # infile = open(filename, 'rb')
    # loaded_raw_sols = pickle.load(infile)
    # print(loaded_raw_sols)
    # infile.close()
    np.savetxt("results.csv", results_summary, delimiter=",")