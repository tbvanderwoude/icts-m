class FlowEdge:
    __slots__ = ["u", "v", "c", "rev"]

    def __init__(self, u, v, c, rev):
        self.u = u
        self.v = v
        self.c = c
        self.rev = rev


class FlowNode:
    def __init__(self, id):
        self.id = id
        self.edges = []

    def add_edge(self, other_id, capacity):
        self.edges.append(FlowEdge(self.id, other_id, capacity, False))
        self.edges.append(FlowEdge(other_id, self.id, capacity, True))


def edmonds_karp(graph, s, t):
    return graph


if __name__ == "__main__":
    print("Hello")
