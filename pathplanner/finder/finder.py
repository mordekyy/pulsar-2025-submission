from math import hypot
from heapq import heappop, heappush
from config import MovementMode
from convert import directions_from_movement_mode


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
    start = tuple(start)
    end = tuple(end)

    dirs = directions_from_movement_mode(MovementMode.FOUR_DIRECTIONS)

    ROW, COL = len(grid), len(grid[0])

    g = {start: 0.0}
    parent = {start: start}  # TODO: handle parents
    f0 = g[start] + remaining_path(*start, end)

    openq = [(f0, start)]  # todo: add children here

    visited = set()

    while openq:
        _, cur = heappop(openq)
        if cur in visited:
            continue
        if cur == end:
            if start == end:
                return [start]
            return None

        visited.add(cur)

        r, c = cur
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if not is_valid(nr, nc) or not is_unblocked(nr, nc) or (nr, nc) in visited:
                continue
            ng = g[cur] + 1.0
            if ng < g.get((nr, nc), float("inf")):
                g[(nr, nc)] = ng
                parent[(nr, nc)] = cur
                f = ng + remaining_path(nr, nc, end)
                heappush(openq, (f, (nr, nc)))

    return None
