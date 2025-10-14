from math import sqrt, hypot, atan2, degrees
from heapq import heappop, heappush
from config import MovementMode, FIELD_CONFIG, ROBOT_CONFIG
from finder.convert import directions_from_movement_mode
from finder.messaging import SearchStepCallback, emit_step


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


def is_unblocked(grid, curr, nxt):
    (r, c), (nr, nc) = curr, nxt
    dz = abs(grid[nr][nc] - grid[r][c])
    dr, dc = abs(nr - r), abs(nc - c)
    if dr == 0 and dc == 0:
        return True
    horiz = FIELD_CONFIG.PIXEL_SIZE_M * (sqrt(2.0) if dr and dc else 1.0)
    angle = degrees(atan2(dz, horiz))
    return angle <= ROBOT_CONFIG.MAX_SLOPE_DEG


def is_destination(row, col, dest):
    if (row != dest[0]):
        return False
    if (col != dest[1]):
        return False
    return True


def remaining_path(row, col, dest):
    return hypot(row - dest[0], col - dest[1]) * ROBOT_CONFIG.REMAINING_DISTANCE_WEIGHT


def a_star(
    grid,
    start,
    end,
    movement_mode=MovementMode.FOUR_DIRECTIONS,
    cost_map=None,
    on_step: SearchStepCallback = None,
):
    start = tuple(start)
    end = tuple(end)
    ROW, COL = len(grid), len(grid[0])

    blocked_cells = []

    if not is_valid(start[0], start[1], ROW, COL):
        return None, blocked_cells
    if not is_valid(end[0], end[1], ROW, COL):
        return None, blocked_cells
    if start == end:
        return [start], blocked_cells

    dirs = directions_from_movement_mode(movement_mode)

    g = {start: 0.0}
    parent = {start: start}
    f0 = g[start] + remaining_path(*start, end)
    openq = [(f0, start)]
    visited = set()
    step_index = 0

    while openq:
        _, cur = heappop(openq)
        if cur in visited:
            continue
        if cur == end:
            if on_step:
                visited_snapshot = set(visited)
                visited_snapshot.add(cur)
                emit_step(on_step, step_index, cur,
                          visited_snapshot, openq, blocked_cells)
            path = []
            node = cur
            while node != start:
                path.append(node)
                node = parent[node]
            path.append(start)
            path.reverse()

            return path, blocked_cells

        visited.add(cur)
        r, c = cur
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if not is_valid(nr, nc, ROW, COL):
                blocked_cells.append((nr, nc))
                continue
            if not is_unblocked(grid, (r, c), (nr, nc)):
                blocked_cells.append((nr, nc))
                continue
            if (nr, nc) in visited:
                continue
            base_step = sqrt(2.0) if (dr and dc) else 1.0
            cost_multiplier = 1.0
            if cost_map is not None:
                cost_multiplier = cost_map[nr][nc]
            step = base_step * cost_multiplier
            ng = g[cur] + step
            if ng < g.get((nr, nc), float("inf")):
                g[(nr, nc)] = ng
                parent[(nr, nc)] = cur
                f = ng + remaining_path(nr, nc, end)
                heappush(openq, (f, (nr, nc)))
        emit_step(on_step, step_index, cur, visited, openq, blocked_cells)
        step_index += 1

    return None, blocked_cells
