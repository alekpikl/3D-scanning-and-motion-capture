import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

def parse_line(line):
    clean_line = line.replace("\n", "")
    point_values = clean_line.split(" ")
    points = [float(p) for p in point_values]

    return points


def read_points(file_path):
    with open(file_path) as f:
        lines = f.readlines()
        points = [parse_line(l) for l in lines]

    points_x, points_y, points_z = zip(*points)

    return list(points_x), list(points_y), list(points_z)

def generate_data(a, b, c):
    x = y = np.arange(-5, 5, 1.0)
    X, Y = np.meshgrid(x, y)
    Z = (X**2/a-Y**2/b)/c

    return X, Y, Z

def visualize(x, y, z, a, b, c):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_zlim(-100, 100)

    sx, sy, sz = generate_data(a,b,c)
    ax.plot_surface(sx, sy, sz, alpha=0.9, rstride=1, cstride=1, linewidth=1, cmap=cm.summer)
    ax.scatter(x, y, z, cmap=cm.summer)

    plt.show()

def main(args):
    points_x, points_y, points_z = read_points(args.data_path + "/points_surface.txt")
    visualize(points_x, points_y, points_z, args.a, args.b, args.c)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_path", type=str, default="data", help="Directory of the data files.")
    parser.add_argument("--a", type=float, default=1.0, help="The a value from Ceres.")
    parser.add_argument("--b", type=float, default=1.0, help="The b value from Ceres.")
    parser.add_argument("--c", type=float, default=1.0, help="The c value from Ceres.")

    main(parser.parse_args())