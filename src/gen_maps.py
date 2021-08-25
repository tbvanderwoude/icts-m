import pathlib
import shutil

from tqdm import tqdm
from gen.map_generator import MapGenerator

if __name__ == "__main__":
    raw_dir = pathlib.Path("new-maps")
    if raw_dir.exists():
        shutil.rmtree(raw_dir)
    raw_dir.mkdir()
    map_generator = MapGenerator("new-maps")
    for K in [1, 3]:
        for k in tqdm(range(1, 17)):
            map_generator.generate_even_batch(
                200,  # number of maps
                20,
                20,  # size
                k,  # number of agents
                K,  # number of teams
                prefix="25",
                min_goal_distance=0,
                open_factor=0.65,
                max_neighbors=3,
            )
            map_generator.generate_even_batch(
                200,  # number of maps
                20,
                20,  # size
                k,  # number of agents
                K,  # number of teams
                prefix="75",
                min_goal_distance=0,
                open_factor=0.65,
                max_neighbors=1,
            )
