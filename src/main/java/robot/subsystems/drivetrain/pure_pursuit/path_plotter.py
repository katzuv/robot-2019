import sys
from typing import List, Tuple

import matplotlib.pyplot as plt


def waypoint_location(waypoint: str) -> Tuple[float, float]:
    """
    :param waypoint: a waypoint in the path
    :return: the x and y coordinate of :waypoint:
    """
    x_start = waypoint.find('x') + 2  # x={first digit}
    y_start = waypoint.find('y') + 2
    x_value = float(waypoint[x_start: y_start - 6].strip())
    y_value = float(waypoint[y_start:].strip())
    return x_value, y_value


def xs_and_ys(waypoints: List[str]) -> Tuple[List[float], List[float]]:
    """
    :param waypoints: all waypoints in the path
    :return: all x-s and y-s coordinates of the waypoints
    """
    xs = []
    ys = []
    for waypoint in waypoints:
        x, y = waypoint_location(waypoint)
        xs.append(x)
        ys.append(y)
    return xs, ys


if __name__ == '__main__':
    first_waypoint = input('Insert the outputted string from the Java program, press Ctrl+D on Linux or Ctrl+Z on Windows to stop input, then hit press Enter:\n')
    points = [first_waypoint] + sys.stdin.readlines()
    plt.scatter(*xs_and_ys(points))
    plt.show()
