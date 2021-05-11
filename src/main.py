import sys

from pprint import pprint

import requests
from mapfmclient import MapfBenchmarker, Problem, MarkedLocation

from mapfm.solve import solve


def run_custom(token, p_id):
    headers = {"X-API-Token": token}
    data = {"algorithm": "ICTS", "version": "0.1.2", "debug": True}
    r = requests.post(
        "https://mapf.nl/api/benchmark/{}".format(p_id), headers=headers, json=data
    )
    assert r.status_code == 200, print(r.content)
    j = r.json()
    problem_json = j["problems"][0]
    problem = Problem(
        problem_json["grid"],
        problem_json["width"],
        problem_json["height"],
        [MarkedLocation.from_dict(i) for i in problem_json["starts"]],
        [MarkedLocation.from_dict(i) for i in problem_json["goals"]],
        0,
        problem_json["id"],
        0,
    )
    solution = solve(problem)
    pprint(solution.serialize())


def str_to_bool(s):
    return s == "true" or s == "True"


if __name__ == "__main__":
    token = "FXJ8wNVeWh4syRdh"
    p_id = int(sys.argv[1])
    profile = str_to_bool(sys.argv[2])
    debug = str_to_bool(sys.argv[3])

    if profile:
        run_custom(token, p_id)
    else:
        benchmark = MapfBenchmarker(
            token=token,
            problem_id=p_id,
            algorithm="ICTS-M",
            version="0.0.1",
            debug=debug,
            solver=solve,
            cores=8,
        )
        benchmark.run()
