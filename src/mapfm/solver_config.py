class SolverConfig:
    __slots__ = [
        "combs",
        "prune",
        "enhanced",
        "id",
        "conflict_avoidance",
        "enumerative",
        "debug",
        "sort_matchings",
        "pruned_child_gen",
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
        sort_matchings: bool = True,
    ):
        self.combs = combs
        self.prune = prune
        self.enhanced = enhanced
        self.id = id
        self.conflict_avoidance = conflict_avoidance
        self.enumerative = enumerative
        self.sort_matchings = sort_matchings
        self.pruned_child_gen = pruned_child_gen
        self.debug = debug
