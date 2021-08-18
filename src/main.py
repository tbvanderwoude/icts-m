import sys

import requests
from mapfmclient import MapfBenchmarker, Problem, MarkedLocation

from bbsolver import solve_bb, solve_bb_api
from ictsm.solve import solve_api, solve_api_enum


def run_custom(token, p_id):
    headers = {"X-API-Token": token}
    data = {"algorithm": "ICTS", "version": "0.2.2", "debug": True}
    r = requests.get(
        "https://mapf.nl/api/benchmark/{}".format(p_id), headers=headers, json=data
    )
    assert r.status_code == 200, print(r.content)
    j = r.json()
    # print(j)
    problem_json = j
    problem = Problem(
        problem_json["grid"],
        problem_json["width"],
        problem_json["height"],
        [MarkedLocation.from_dict(i) for i in problem_json["starts"]],
        [MarkedLocation.from_dict(i) for i in problem_json["goals"]],
    )
    solution = solve_bb(problem)
    # pprint(solution.serialize())


def str_to_bool(s):
    return s == "true" or s == "True"

import json

if __name__ == "__main__":
    secrets_filename = 'token'
    with open(secrets_filename, 'r') as f:
        tokens = json.loads(f.read())
    token = tokens['SERVER_TOKEN']
    p_id = int(sys.argv[1])
    profile = str_to_bool(sys.argv[2])
    debug = str_to_bool(sys.argv[3])
    enumerative = str_to_bool(sys.argv[4])
    if profile:
        run_custom(token, p_id)
    else:
        if enumerative:
            benchmark = MapfBenchmarker(
                token=token,
                benchmark=p_id,
                algorithm="ICTS (branch-and-bound)",
                version="0.2.7",
                debug=debug,
                solver=solve_bb_api,
                cores=8,
            )
            benchmark.run()
        else:
            benchmark = MapfBenchmarker(
                token=token,
                benchmark=p_id,
                algorithm="ICTS-M",
                version="0.0.2",
                debug=debug,
                solver=solve_api,
                cores=8,
            )
            benchmark.run()
