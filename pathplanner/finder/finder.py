from math import hypot


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
