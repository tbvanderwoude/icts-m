Byte = 1
KiloByte = Byte * 1024
MegaByte = KiloByte * 1024
GigaByte = MegaByte * 1024


class SolverConfig:
    __slots__ = [
        "name",
        "combs",
        "prune",
        "enhanced",
        "id",
        "conflict_avoidance",
        "enumerative",
        "debug",
        "budget_search",
        "sort_matchings",
        "pruned_child_gen",
        "mem_limit",
    ]

    def __init__(
        self,
        combs: int,
        prune: bool,
        enhanced: bool,
        pruned_child_gen: bool,
        id: bool,
        conflict_avoidance: bool,
        enumerative: bool,
        debug: bool,
        budget_search: bool = True,
        sort_matchings: bool = True,
        name: str = "unnamed",
        mem_limit: float = 3.0,
    ):
        self.name = name
        self.combs = combs
        self.prune = prune
        self.enhanced = enhanced
        self.id = id
        self.conflict_avoidance = conflict_avoidance
        self.enumerative = enumerative
        self.budget_search = budget_search
        self.sort_matchings = sort_matchings
        self.pruned_child_gen = pruned_child_gen
        self.debug = debug
        self.mem_limit = int(mem_limit * GigaByte)
