from .util import index_path


class IDContext:
    def __init__(self, other_agents, paths, lens):
        self.other_agents = other_agents
        self.paths = paths
        self.lens = lens
        self.len = 0
        self.path = []
        self.cache = dict()

    def sample_context_node(self, depth: int):
        while depth >= self.len:
            self.path.append(
                list(
                    map(
                        lambda agent: index_path(
                            self.paths[agent], self.len, self.lens[agent]
                        ),
                        self.other_agents,
                    )
                )
            )
            self.len += 1
        return self.path[depth]
