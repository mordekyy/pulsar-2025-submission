from math import sqrt, hypot, atan2, degrees
from heapq import heappop, heappush
from itertools import count
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

    blocked_cells: list[tuple[int, int]] = []
    blocked_seen: set[tuple[int, int]] = set()

    if not is_valid(start[0], start[1], ROW, COL):
        return None, blocked_cells
    if not is_valid(end[0], end[1], ROW, COL):
        return None, blocked_cells
    if start == end:
        return [start], blocked_cells

    dirs = directions_from_movement_mode(movement_mode)

    tolerance = ROBOT_CONFIG.END_TOLERANCE_PX
    end_row, end_col = end
    row_min = max(0, end_row - tolerance)
    row_max = min(ROW - 1, end_row + tolerance)
    col_min = max(0, end_col - tolerance)
    col_max = min(COL - 1, end_col + tolerance)

    goal_cells = set()
    tol_sq = tolerance * tolerance
    for r in range(row_min, row_max + 1):
        for c in range(col_min, col_max + 1):
            dr = r - end_row
            dc = c - end_col
            if dr * dr + dc * dc <= tol_sq:
                goal_cells.add((r, c))
    if not goal_cells:
        goal_cells.add(end)

    forward_open: list[tuple[float, int, tuple[int, int]]] = []
    backward_open: list[tuple[float, int, tuple[int, int]]] = []

    order = count()

    def push(queue: list[tuple[float, int, tuple[int, int]]], f_cost: float, node: tuple[int, int]) -> None:
        heappush(queue, (f_cost, next(order), node))

    forward_g = {start: 0.0}
    backward_g: dict[tuple[int, int], float] = {}
    parent_forward = {start: start}
    parent_backward: dict[tuple[int, int], tuple[int, int]] = {}
    visited_forward: set[tuple[int, int]] = set()
    visited_backward: set[tuple[int, int]] = set()
    visited_union: set[tuple[int, int]] = set()

    push(forward_open, remaining_path(*start, end), start)
    for cell in goal_cells:
        backward_g[cell] = 0.0
        parent_backward[cell] = cell
        push(backward_open, remaining_path(*cell, start), cell)

    step_index = 0
    meet_node: tuple[int, int] | None = None

    def emit_snapshot(current: tuple[int, int], is_goal: bool) -> None:
        if not on_step:
            return
        open_nodes = [node for _, _, node in forward_open]
        open_nodes.extend(node for _, _, node in backward_open)
        emit_step(
            on_step,
            step_index,
            current,
            visited_union,
            [(0.0, node) for node in open_nodes],
            blocked_cells,
            is_goal=is_goal,
        )

    while forward_open and backward_open:
        use_forward = forward_open[0][0] <= backward_open[0][0]
        open_queue = forward_open if use_forward else backward_open
        visited_self = visited_forward if use_forward else visited_backward
        visited_other = visited_backward if use_forward else visited_forward
        g_self = forward_g if use_forward else backward_g
        parent_self = parent_forward if use_forward else parent_backward
        target = end if use_forward else start

        while open_queue:
            _, _, current = heappop(open_queue)
            if current in visited_self:
                continue
            break
        else:
            break

        visited_self.add(current)
        if current not in visited_union:
            visited_union.add(current)

        if current in goal_cells:
            meet_node = current
            emit_snapshot(current, True)
            break

        if current in visited_other:
            meet_node = current
            emit_snapshot(current, True)
            break

        r, c = current
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if not is_valid(nr, nc, ROW, COL):
                if (nr, nc) not in blocked_seen:
                    blocked_seen.add((nr, nc))
                    blocked_cells.append((nr, nc))
                continue
            if not is_unblocked(grid, (r, c), (nr, nc)):
                if (nr, nc) not in blocked_seen:
                    blocked_seen.add((nr, nc))
                    blocked_cells.append((nr, nc))
                continue
            nxt = (nr, nc)
            if nxt in visited_self:
                continue
            base_step = sqrt(2.0) if (dr and dc) else 1.0
            cost_multiplier = 1.0
            if cost_map is not None:
                cost_multiplier = cost_map[nr][nc]
            step_cost = base_step * cost_multiplier
            tentative_g = g_self[current] + step_cost
            if tentative_g < g_self.get(nxt, float("inf")):
                g_self[nxt] = tentative_g
                parent_self[nxt] = current
                f_score = tentative_g + remaining_path(nr, nc, target)
                push(open_queue, f_score, nxt)

        emit_snapshot(current, False)
        step_index += 1

    if meet_node is None:
        return None, blocked_cells

    path_forward: list[tuple[int, int]] = []
    node = meet_node
    while node != start:
        path_forward.append(node)
        node = parent_forward[node]
    path_forward.append(start)
    path_forward.reverse()

    path_backward: list[tuple[int, int]] = []
    node = meet_node
    while True:
        parent_node = parent_backward.get(node, node)
        if parent_node == node:
            break
        node = parent_node
        path_backward.append(node)

    return path_forward + path_backward, blocked_cells
