from math import hypot
from heapq import heappop


def is_valid(row, col, ROW, COL):
    if (row < 0):
        return False
    if (row >= ROW):
        return False
    if (col < 0):
        return False
    if (col >= COL):
        return False
    return True


def is_unblocked(grid, row, col):
    return grid[row][col] == 1


def is_destination(row, col, dest):
    if (row != dest[0]):
        return False
    if (col != dest[1]):
        return False
    return True


def remaining_path(row, col, dest):
    return hypot(row - dest[0], col - dest[1])


def a_star(grid, start, end):
    g = {start: 0.0}
    parent = {start: start}  # TODO: handle parents
    f0 = g[start] + remaining_path(*start, end)

    openq = [(f0, start)]  # todo: add children here

    closed = set()

    while openq:
        _, cur = heappop(openq)
        if cur in closed:
            continue
        if cur == end:
            if start == end:
                return [start]
            return None  # am terminat aici
        closed.add(cur)

    return None
