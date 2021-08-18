import heapq
from typing import List, Tuple, Set, Generator

from mapfmclient import Problem, Solution

from ictsm.astar import astar
from ictsm.maze import Maze
from branch_and_bound.bbnode import BBNode
from branch_and_bound.assignment_solver import solve_problem
from ictsm.compact_location import MarkedCompactLocation, compact_location, expand_location
from branch_and_bound.assignment_problem import AssignmentProblem
from ictsm.solver import Solver
from ictsm.solver_config import SolverConfig


def evaluate(node: BBNode):
    return node.lower_bound + node.__hash__() % 100

def murty_gen(costs, root):
    ls: List[BBNode] = [root]
    heapq.heapify(ls)
    seen: Set[BBNode] = set()
    while ls:
        n: BBNode = heapq.heappop(ls)
        if n not in seen: # and (not min_cost or n.lower_bound < min_cost):
            seen.add(n)
            if n.is_leaf():
                # c = evaluate(n)
                # print("{} Leaf node with lower-bound {} evaluated to {} (current upper: {})".format(index,n.lower_bound, c,
                #                                                                                  min_cost))
                yield n
                # index+=1
                # if not min_cost or c < min_cost:
                #     min_cost = c
            else:
                children = n.problem.generate_subproblems()
                if len(children) == 1:
                    heapq.heappush(ls, BBNode(n, children[0], n.lower_bound))
                else:
                    for sub_problem in n.problem.generate_subproblems():
                        sub_cost = solve_problem(costs, sub_problem)
                        heapq.heappush(ls, BBNode(n, sub_problem, sub_cost))

def solve_bb_api(problem: Problem):
    return solve_bb(problem)[0]

def solve_bb(problem: Problem):
    paths: List[List[Tuple[int, int]]] = []
    # translates MAPFM problem to assignment problem (relaxation)
    k = len(problem.starts)
    costs = [[0 for _ in range(k)] for _ in range(k)]
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
    for (i,(al,ac)) in enumerate(agents):
        for (j,(gl,gc)) in enumerate(goals):
            if ac == gc:
                shortest_path = astar(maze, al, gl)
                c = len(shortest_path) - 1
                costs[i][j] = c

    team_id = [x[1] for x in agents]
    team_tasks =[set([g[0] for g in enumerate(goals) if g[1][1] == team]) for team in range(K)]
    root_problem = AssignmentProblem(team_id, team_tasks, K, k, k)
    print("Computing BB root cost")
    root_cost = solve_problem(costs, root_problem)
    root = BBNode(None, root_problem, root_cost)
    matching_generator = murty_gen(costs, root)

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
    solver = Solver(config, problem)
    min_sol = None
    ub = -1
    print("Generating bb nodes")
    for bb_node in matching_generator:
        if min_sol and bb_node.lower_bound >= min_sol.sic:
            break
        print(bb_node.problem.assignments,bb_node.lower_bound,ub)
        team_goals = dict(map(lambda x: (x[0], {goals[x[1]][0]}), enumerate(bb_node.problem.assignments)))
        sol = solver.solve_tapf_instance(matching_agents, team_agent_indices, team_goals)
        if sol:
            if not min_sol or min_sol.sic > sol.sic:
                min_sol = sol
                if solver.config.budget_search:
                    ub = min_sol.sic
                    solver.update_budget(min_sol.sic)
    if min_sol:
        # print("Min sol was found")
        subsols = list(zip(*min_sol.solution))
        # print(indices)
        # indexed_subsols = list(enumerate(subsols))
        # indexed_subsols.sort(key=lambda x: index_map[x[0]])
        # print(indexed_subsols)
        # print(indexed_subsols)
        for (i, subsol) in enumerate(subsols):
            paths.append(list(map(lambda loc: expand_location(loc), subsol)))
        return (
            Solution.from_paths(paths),
            solver.ict_searcher.max_delta,
            solver.max_k_solved,
            min_sol.sic
        )
    else:
        return None, solver.ict_searcher.max_delta, solver.max_k_solved, None