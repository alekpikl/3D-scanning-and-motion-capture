import argparse

import matplotlib.pyplot as plt
import matplotlib.transforms as T
import numpy as np

# Some magic
m=np.sin
D=np.cos
O=np.stack
N=np.radians
w=np.array

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

    return np.array(list(points_x)), np.array(list(points_y))


def transform_points(x, y, a, b, c):
    d=N(-a)
    W=O((x,y),axis=-1).dot(w(((D(d),-m(d)),(m(d),D(d)))))+w([b,c])
    return W[:,0],W[:,1]


def main(args):
    x_0, y_0 = transform_points(*(read_points(args.data_path + "/points_dragon_1.txt")), args.deg, args.tx, args.ty)
    x_1, y_1 = read_points(args.data_path + "/points_dragon_2.txt")

    fig, ax = plt.subplots()
    image = plt.imread(args.data_path + "/dragon.png")

    image = np.flipud(image)
    plt.axis([-600, 2000, -500, 1200])
    im = plt.imshow(image, origin="lower", alpha=0.6)
    center = T.Affine2D().translate(-image.shape[1] / 2, -image.shape[0] / 2)
    trans_data = center + ax.transData
    im.set_transform(trans_data)

    im2 = plt.imshow(image, origin="lower", alpha=0.6)
    trans_data2 = center.rotate_deg(args.deg).translate(args.tx, args.ty) + ax.transData
    im2.set_transform(trans_data2)

    plt.scatter(x_0, y_0, s=3, c="b")
    plt.scatter(x_1, y_1, s=3, c="r")

    for px_0, py_0, px_1, py_1 in zip(x_0, y_0, x_1, y_1):
        plt.plot([px_0, px_1], [py_0, py_1], c="g", alpha=0.1)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_path", type=str, default="../../data", help="Directory of the data files.")
    parser.add_argument("--deg", type=float, default=0.0, help="Rotation value in degree from Ceres.")
    parser.add_argument("--tx", type=float, default=0.0, help="x translation value from Ceres.")
    parser.add_argument("--ty", type=float, default=0.0, help="y translation value from Ceres.")

    main(parser.parse_args())
