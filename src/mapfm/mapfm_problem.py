from typing import Dict, List


class MAPFMProblem:
    __slots__ = [
        "agents",
        "team_agent_indices",
        "team_goals",
        "k",
        "team_lens",
        "n_teams",
        "teams",
    ]
    def __init__(self,agents,team_agent_indices: Dict[int, List[int]],team_goals):
        self.agents = agents
        self.team_agent_indices = team_agent_indices
        self.team_goals = team_goals
        self.k = len(agents)
        self.n_teams = len(team_agent_indices)
        self.team_lens = dict()
        for team in team_goals:
            self.team_lens[team] = len(team_goals[team])
        self.teams = list(self.team_agent_indices.keys())