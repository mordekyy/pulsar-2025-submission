from math import sqrt
from math import hypot
from heapq import heappop, heappush
from config import MovementMode
from finder.convert import directions_from_movement_mode


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
    return True
    return grid[row][col] == 1


def is_destination(row, col, dest):
    if (row != dest[0]):
        return False
    if (col != dest[1]):
        return False
    return True


def remaining_path(row, col, dest):
    return hypot(row - dest[0], col - dest[1])


def a_star(grid, start, end, movement_mode=MovementMode.FOUR_DIRECTIONS):
    start = tuple(start)
    end = tuple(end)

    dirs = directions_from_movement_mode(movement_mode)

    ROW, COL = len(grid), len(grid[0])

    if not (0 <= start[0] < ROW and 0 <= start[1] < COL):
        return None
    if not (0 <= end[0] < ROW and 0 <= end[1] < COL):
        return None
    if grid[start[0]][start[1]] != 1:
        return None
    if grid[end[0]][end[1]] != 1:
        return None
    if start == end:
        return [start]

    g = {start: 0.0}
    parent = {start: start}
    f0 = g[start] + remaining_path(*start, end)

    openq = [(f0, start)]

    visited = set()

    while openq:
        _, cur = heappop(openq)
        if cur in visited:
            continue
        if cur == end:
            path = []
            node = cur
            while node != start:
                path.append(node)
                node = parent[node]
            path.append(start)
            path.reverse()
            return path

        visited.add(cur)

        r, c = cur
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if not is_valid(nr, nc, ROW, COL) or not is_unblocked(grid, nr, nc) or (nr, nc) in visited:
                continue
            step = sqrt(2.0) if len(dirs) > 4 else 1.0
            ng = g[cur] + step
            if ng < g.get((nr, nc), float("inf")):
                g[(nr, nc)] = ng
                parent[(nr, nc)] = cur
                f = ng + remaining_path(nr, nc, end)
                heappush(openq, (f, (nr, nc)))

    return None
