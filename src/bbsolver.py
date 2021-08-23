import heapq
from copy import copy
from typing import List, Tuple, Set, Generator

from mapfmclient import Problem, Solution, MarkedLocation

from ictsm.astar import astar
from ictsm.maze import Maze
from branch_and_bound.bbnode import BBNode
from branch_and_bound.assignment_solver import solve_problem
from ictsm.compact_location import MarkedCompactLocation, compact_location, expand_location
from branch_and_bound.assignment_problem import AssignmentProblem
from ictsm.solver import Solver
from ictsm.solver_config import SolverConfig

def murty_gen(costs, root):
    ls: List[BBNode] = [root]
    heapq.heapify(ls)
    seen: Set[BBNode] = set()
    while ls:
        n: BBNode = heapq.heappop(ls)
        if n not in seen:
            seen.add(n)
            if n.is_leaf():
                yield n
            else:
                children = n.problem.generate_subproblems()
                if len(children) == 1:
                    heapq.heappush(ls, BBNode(n, children[0], n.lower_bound))
                else:
                    for sub_problem in n.problem.generate_subproblems():
                        sub_cost = solve_problem(costs, sub_problem)
                        heapq.heappush(ls, BBNode(n, sub_problem, sub_cost))
        else:
            print("Already seen?")

def solve_bb_api(problem: Problem):
    return solve_bb(problem)

def create_root(agents, goals, costs, K, k) -> BBNode:
    team_id = [x[1] for x in agents]
    team_tasks =[set([g[0] for g in enumerate(goals) if g[1][1] == team]) for team in range(K)]
    root_problem = AssignmentProblem(team_id, team_tasks, K, k, k)
    print("Computing BB root cost")
    root_cost = solve_problem(costs, root_problem)
    root = BBNode(None, root_problem, root_cost)
    return root


def mapf_problem_from_assignment():
    pass


def solve_bb(problem: Problem):
    # translates MAPFM problem to assignment problem (relaxation)
    k = len(problem.starts)
    # makes sure that the K teams are numbered without gaps as 0...(K-1)
    reverse_map = enumerate(sorted(set(map(lambda x: x.color, problem.starts))))
    color_map = dict([(sub[1], sub[0]) for sub in reverse_map])
    K = len(color_map)
    agents: List[MarkedCompactLocation] = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), color_map[marked.color]),
            problem.starts,
        )
    )
    goals: List[MarkedCompactLocation] = list(
        map(
            lambda marked: (compact_location(marked.x, marked.y), color_map[marked.color]),
            problem.goals,
        )
    )
    maze: Maze = Maze(problem.grid, problem.width, problem.height)
    print("Computing shortest paths")
    costs = [[0 for _ in range(k)] for _ in range(k)]
    for (i,(al,ac)) in enumerate(agents):
        for (j,(gl,gc)) in enumerate(goals):
            if ac == gc:
                shortest_path = astar(maze, al, gl)
                c = len(shortest_path) - 1
                costs[i][j] = c

    root: BBNode = create_root(agents,goals,costs,K,k)
    matching_agents = list(map(lambda x: (x[1][0], x[0]), enumerate(agents)))
    team_agent_indices = dict(map(lambda x: (x[0], {x[0]}), enumerate(agents)))
    print("Creating solver")

    config = SolverConfig(
        combs=3,
        prune=True,
        enhanced=False,
        pruned_child_gen=True,
        id=True,
        conflict_avoidance=True,
        enumerative=True,
        sort_matchings=True,
        debug=False,
        budget_search=True,
    )
    solver = Solver(config)
    min_sic = None
    min_sol = None
    print("Generating bb nodes")

    ls: List[BBNode] = [root]
    heapq.heapify(ls)
    seen: Set[BBNode] = set()

    leaf_p_agents = list(
        map(lambda x: MarkedLocation(color=x[1], x=expand_location(x[0])[0], y=expand_location(x[0])[1]), matching_agents))

    print(leaf_p_agents)

    while ls:
        n: BBNode = heapq.heappop(ls)
        if n not in seen and (not min_sol or n.lower_bound < min_sic):
            seen.add(n)
            if n.is_leaf():
                print(n.problem.assignments, n.lower_bound, min_sic)
                leaf_p_goals = list(
                    map(
                        lambda marked: MarkedLocation(x = marked[1].x, y = marked[1].y,color=marked[0]),
                        zip(n.problem.assignments,problem.goals),
                    )
                )
                leaf_p: Problem = Problem(problem.grid,problem.width,problem.height,copy(leaf_p_agents),copy(leaf_p_goals))
                sol: Solution = solver(leaf_p,min_sic)[0]
                if sol:
                    c: int = compute_sol_cost(sol)
                    if not min_sic or min_sic > c:
                        min_sol = sol
                        if solver.config.budget_search:
                            min_sic = c
            else:
                children = n.problem.generate_subproblems()
                if len(children) == 1:
                    heapq.heappush(ls, BBNode(n, children[0], n.lower_bound))
                else:
                    for sub_problem in n.problem.generate_subproblems():
                        sub_cost = solve_problem(costs, sub_problem)
                        if not min_sol or sub_cost < min_sic:
                            heapq.heappush(ls, BBNode(n, sub_problem, sub_cost))
    return min_sol

def compute_sol_cost(sol: Solution) -> int:
    sic = 0
    for path in sol.paths:
        xs = path.route
        last = xs[-1]
        while xs[-1] == last:
            xs = xs[:-1]
        sic += len(xs)
    return sic