import pathlib
import shutil

from gen.map_generator import MapGenerator

if __name__ == '__main__':
    raw_dir = pathlib.Path("new-maps")
    if raw_dir.exists():
        shutil.rmtree(raw_dir)
    raw_dir.mkdir()
    map_generator = MapGenerator("new-maps")
    for i in range(1, 16):
        map_generator.generate_even_batch(200, 20, 20, i, 1, prefix="Open", min_goal_distance=0, open_factor=1.0, max_neighbors=4)
        map_generator.generate_even_batch(200, 20, 20, i, 3, prefix="Open", min_goal_distance=0, open_factor=1.0,max_neighbors=4)