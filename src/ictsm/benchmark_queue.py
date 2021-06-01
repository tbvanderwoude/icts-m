class BenchmarkQueue:
    def __init__(self, name):
        self.name = name
        with open(name, "a"):
            pass

    def get_next(self):
        with open(self.name, "r") as f:
            return f.readline().strip()

    def completed(self):
        with open(self.name, "r") as fin:
            data = fin.read().splitlines(True)
        with open(self.name, "w") as fout:
            fout.writelines(data[1:])

    def add(self, data):
        with open(self.name, "a") as f:
            f.write(data + "\n")