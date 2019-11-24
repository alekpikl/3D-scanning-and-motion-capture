import argparse
from scipy.stats import norm
import matplotlib.pyplot as plt

def parse_line(line):
    clean_line = line.replace("\n", "")
    point_values = clean_line.split(" ")
    points = [float(p) for p in point_values]

    return points


def read_points(file_path):
    with open(file_path) as f:
        lines = f.readlines()
        points = [parse_line(l) for l in lines]

    points_x, points_y = zip(*points)

    return list(points_x), list(points_y)


def visualize(points_x, points_y, mu, sigma):
    plt.plot(points_x, [norm.pdf(x, mu, sigma) for x in points_x], c="r")
    plt.scatter(points_x, points_y, s=1, c="b")
    plt.show()


def main(args):
    points_x, points_y = read_points(args.data_path + "/points_gaussian.txt")
    visualize(points_x, points_y, args.mu, args.sigma)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_path", type=str, default="../../data", help="Directory of the data files.")
    parser.add_argument("--mu", type=float, default=0.0, help="The mu value from Ceres.")
    parser.add_argument("--sigma", type=float, default=1.0, help="The sigma value from Ceres.")

    main(parser.parse_args())
