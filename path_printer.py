import numpy as np
import random
import matplotlib.pyplot as plt

def read_paths(file_name):
    paths = []
    with open(file_name, 'r') as file:
        for line in file:
            if line.startswith("Agent"):
                path = line.split(":")[1].strip().split("->")
                path = [point.strip("()") for point in path]
                paths.append(path)
    return paths

def load_pgm_map(file_name):
    with open(file_name, 'rb') as f:
        # Read header
        assert f.readline() == b'P5\n'
        while True:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            else:
                width, height = map(int, line.split())
                break
        max_val = int(f.readline().strip())
        assert max_val == 255

        # Read binary data
        map_data = np.fromfile(f, dtype=np.uint8).reshape((height, width))
        return map_data, height, width

def plot_paths(paths_file, map_file):
    paths = read_paths(paths_file)
    map_data, rows, columns = load_pgm_map(map_file)
    matrix = np.ones((rows, columns, 3))  # Create a white matrix

    # Set obstacles in the matrix
    for i in range(rows):
        for j in range(columns):
            if map_data[i, j] != 254:  # Assuming 254 is free space and the rest is an obstacle
                matrix[i, j] = [0, 0, 0]  # Set obstacle color to black

    colors = []
    for _ in range(len(paths)):
        colors.append((random.random(), random.random(), random.random()))

    for path_index, path in enumerate(paths):
        for point in path:
            if point:  # Ensure the point is not an empty string
                x, y = map(int, point.split(','))
                matrix[x, y] = colors[path_index]

    plt.imshow(matrix)
    plt.show()

if __name__ == "__main__":
    paths_file = input("Enter the name of the paths file: ")
    map_file = input("Enter the name of the .pgm map file: ")
    plot_paths(paths_file, map_file)