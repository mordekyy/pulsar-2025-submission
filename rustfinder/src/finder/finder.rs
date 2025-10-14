use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashSet};
use std::f32::consts::SQRT_2;

use ndarray::Array2;
use ordered_float::NotNan;

use crate::config::{FieldConfig, MovementMode, RobotConfig};

use super::messaging::{emit_step, SearchStepCallback};

pub type Grid = Array2<f32>;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
struct QueueNode {
    f_cost: NotNan<f32>,
    index: usize,
}

impl QueueNode {
    fn new(f_cost: f32, index: usize) -> Self {
        let f = NotNan::new(f_cost).unwrap_or_else(|_| NotNan::new(f32::INFINITY).unwrap());
        Self { f_cost: f, index }
    }
}

impl Ord for QueueNode {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.f_cost.cmp(&other.f_cost) {
            Ordering::Equal => self.index.cmp(&other.index),
            ord => ord,
        }
    }
}

impl PartialOrd for QueueNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn is_valid(row: isize, col: isize, rows: usize, cols: usize) -> bool {
    row >= 0 && row < rows as isize && col >= 0 && col < cols as isize
}

fn remaining_path(robot_config: &RobotConfig, row: usize, col: usize, dest: (usize, usize)) -> f32 {
    let (dr, dc) = (
        row as f32 - dest.0 as f32,
        col as f32 - dest.1 as f32,
    );
    dr.hypot(dc) * robot_config.remaining_distance_weight
}

#[inline]
fn to_index(row: usize, col: usize, cols: usize) -> usize {
    row * cols + col
}

#[inline]
fn from_index(index: usize, cols: usize) -> (usize, usize) {
    (index / cols, index % cols)
}

fn is_unblocked(
    grid: &Grid,
    current: (usize, usize),
    next: (usize, usize),
    field_config: &FieldConfig,
    robot_config: &RobotConfig,
) -> bool {
    let (r, c) = current;
    let (nr, nc) = next;
    if (r, c) == (nr, nc) {
        return true;
    }
    let dz = (grid[(nr, nc)] - grid[(r, c)]).abs();
    let dr = (nr as isize - r as isize).abs();
    let dc = (nc as isize - c as isize).abs();
    let horiz_scale = if dr != 0 && dc != 0 { SQRT_2 } else { 1.0 };
    let horiz = field_config.pixel_size_m * horiz_scale;
    let angle = dz.atan2(horiz).to_degrees();
    angle <= robot_config.max_slope_deg
}

pub fn a_star<'a>(
    grid: &Grid,
    start: (usize, usize),
    end: (usize, usize),
    movement_mode: MovementMode,
    cost_map: Option<&Grid>,
    field_config: &FieldConfig,
    robot_config: &RobotConfig,
    mut on_step: SearchStepCallback<'a>,
) -> (Option<Vec<(usize, usize)>>, Vec<(isize, isize)>) {
    let rows = grid.nrows();
    let cols = grid.ncols();
    let total_cells = rows * cols;

    let mut blocked_cells: Vec<(isize, isize)> = Vec::new();
    let mut blocked_seen: HashSet<(isize, isize)> = HashSet::with_capacity(rows + cols);

    if !is_valid(start.0 as isize, start.1 as isize, rows, cols) {
        return (None, blocked_cells);
    }
    if !is_valid(end.0 as isize, end.1 as isize, rows, cols) {
        return (None, blocked_cells);
    }
    if start == end {
        return (Some(vec![start]), blocked_cells);
    }

    let tolerance = robot_config.end_tolerance_px as isize;
    let tol_sq = tolerance * tolerance;
    let end_row = end.0 as isize;
    let end_col = end.1 as isize;
    let row_min = (end_row - tolerance).max(0);
    let row_max = (end_row + tolerance).min(rows as isize - 1);
    let col_min = (end_col - tolerance).max(0);
    let col_max = (end_col + tolerance).min(cols as isize - 1);

    let start_idx = to_index(start.0, start.1, cols);
    let end_idx = to_index(end.0, end.1, cols);

    let mut goal_indices: Vec<usize> = Vec::new();
    for r in row_min..=row_max {
        for c in col_min..=col_max {
            let dr = r - end_row;
            let dc = c - end_col;
            if tol_sq == 0 {
                if r == end_row && c == end_col {
                    goal_indices.push(to_index(r as usize, c as usize, cols));
                }
            } else if dr * dr + dc * dc <= tol_sq {
                goal_indices.push(to_index(r as usize, c as usize, cols));
            }
        }
    }
    if goal_indices.is_empty() {
        goal_indices.push(end_idx);
    }
    let goal_set: HashSet<usize> = goal_indices.iter().copied().collect();

    let mut forward_open: BinaryHeap<Reverse<QueueNode>> = BinaryHeap::new();
    let mut backward_open: BinaryHeap<Reverse<QueueNode>> = BinaryHeap::new();
    let mut g_forward: Vec<f32> = vec![f32::INFINITY; total_cells];
    let mut g_backward: Vec<f32> = vec![f32::INFINITY; total_cells];
    let mut parent_forward: Vec<usize> = vec![usize::MAX; total_cells];
    let mut parent_backward: Vec<usize> = vec![usize::MAX; total_cells];
    let mut visited_forward: Vec<bool> = vec![false; total_cells];
    let mut visited_backward: Vec<bool> = vec![false; total_cells];
    let mut visited_union: Vec<bool> = vec![false; total_cells];
    let mut visited_coords: Vec<(usize, usize)> = Vec::with_capacity(total_cells);

    g_forward[start_idx] = 0.0;
    parent_forward[start_idx] = start_idx;
    forward_open.push(Reverse(QueueNode::new(
        remaining_path(robot_config, start.0, start.1, end),
        start_idx,
    )));

    for &goal_idx in &goal_indices {
        g_backward[goal_idx] = 0.0;
        parent_backward[goal_idx] = goal_idx;
        let (gr, gc) = from_index(goal_idx, cols);
        backward_open.push(Reverse(QueueNode::new(
            remaining_path(robot_config, gr, gc, start),
            goal_idx,
        )));
    }

    let directions = movement_mode.directions();
    let mut step_index = 0_usize;
    let mut open_buffer: Vec<(usize, usize)> = Vec::new();
    let mut meet_idx: Option<usize> = None;

    while !forward_open.is_empty() && !backward_open.is_empty() {
        // Expand from the forward frontier
        let meeting_from_forward = loop {
            let Some(Reverse(node)) = forward_open.pop() else {
                break None;
            };
            let current_idx = node.index;
            if visited_forward[current_idx] {
                continue;
            }
            visited_forward[current_idx] = true;
            if !visited_union[current_idx] {
                visited_union[current_idx] = true;
                visited_coords.push(from_index(current_idx, cols));
            }

            if goal_set.contains(&current_idx) {
                if on_step.is_some() {
                    open_buffer.clear();
                    open_buffer.extend(
                        forward_open
                            .iter()
                            .map(|entry| from_index(entry.0.index, cols)),
                    );
                    open_buffer.extend(
                        backward_open
                            .iter()
                            .map(|entry| from_index(entry.0.index, cols)),
                    );
                    emit_step(
                        &mut on_step,
                        step_index,
                        from_index(current_idx, cols),
                        &visited_coords,
                        &open_buffer,
                        &blocked_cells,
                        true,
                    );
                }
                break Some(current_idx);
            }

            if visited_backward[current_idx] {
                if on_step.is_some() {
                    open_buffer.clear();
                    open_buffer.extend(
                        forward_open
                            .iter()
                            .map(|entry| from_index(entry.0.index, cols)),
                    );
                    open_buffer.extend(
                        backward_open
                            .iter()
                            .map(|entry| from_index(entry.0.index, cols)),
                    );
                    emit_step(
                        &mut on_step,
                        step_index,
                        from_index(current_idx, cols),
                        &visited_coords,
                        &open_buffer,
                        &blocked_cells,
                        true,
                    );
                }
                break Some(current_idx);
            }

            let (r, c) = from_index(current_idx, cols);
            for &(dr, dc) in directions.iter() {
                let nr = r as isize + dr;
                let nc = c as isize + dc;
                if !is_valid(nr, nc, rows, cols) {
                    let candidate = (nr, nc);
                    if blocked_seen.insert(candidate) {
                        blocked_cells.push(candidate);
                    }
                    continue;
                }

                let nr_usize = nr as usize;
                let nc_usize = nc as usize;
                let next_idx = to_index(nr_usize, nc_usize, cols);
                let next = (nr_usize, nc_usize);

                if !is_unblocked(grid, (r, c), next, field_config, robot_config) {
                    let candidate = (nr, nc);
                    if blocked_seen.insert(candidate) {
                        blocked_cells.push(candidate);
                    }
                    continue;
                }
                if visited_forward[next_idx] {
                    continue;
                }

                let base_step = if dr != 0 && dc != 0 { SQRT_2 } else { 1.0 };
                let cost_multiplier = cost_map
                    .map(|cm| cm[(nr_usize, nc_usize)])
                    .unwrap_or(1.0);
                let step_cost = base_step * cost_multiplier;
                let tentative_g = g_forward[current_idx] + step_cost;

                if tentative_g < g_forward[next_idx] {
                    g_forward[next_idx] = tentative_g;
                    parent_forward[next_idx] = current_idx;
                    let f_cost =
                        tentative_g + remaining_path(robot_config, nr_usize, nc_usize, end);
                    forward_open.push(Reverse(QueueNode::new(f_cost, next_idx)));
                }
            }

            if on_step.is_some() {
                open_buffer.clear();
                open_buffer.extend(
                    forward_open
                        .iter()
                        .map(|entry| from_index(entry.0.index, cols)),
                );
                open_buffer.extend(
                    backward_open
                        .iter()
                        .map(|entry| from_index(entry.0.index, cols)),
                );
                emit_step(
                    &mut on_step,
                    step_index,
                    from_index(current_idx, cols),
                    &visited_coords,
                    &open_buffer,
                    &blocked_cells,
                    false,
                );
            }
            step_index += 1;
            break None;
        };

        if let Some(idx) = meeting_from_forward {
            meet_idx = Some(idx);
            break;
        }

        if backward_open.is_empty() {
            break;
        }

        // Expand from the backward frontier
        let meeting_from_backward = loop {
            let Some(Reverse(node)) = backward_open.pop() else {
                break None;
            };
            let current_idx = node.index;
            if visited_backward[current_idx] {
                continue;
            }
                visited_backward[current_idx] = true;
                if !visited_union[current_idx] {
                    visited_union[current_idx] = true;
                    visited_coords.push(from_index(current_idx, cols));
                }

                if current_idx == start_idx {
                    if on_step.is_some() {
                        open_buffer.clear();
                        open_buffer.extend(
                            forward_open
                                .iter()
                                .map(|entry| from_index(entry.0.index, cols)),
                        );
                        open_buffer.extend(
                            backward_open
                                .iter()
                                .map(|entry| from_index(entry.0.index, cols)),
                        );
                        emit_step(
                            &mut on_step,
                            step_index,
                            from_index(current_idx, cols),
                            &visited_coords,
                            &open_buffer,
                            &blocked_cells,
                            true,
                        );
                    }
                    break Some(current_idx);
                }

                if visited_forward[current_idx] {
                    if on_step.is_some() {
                        open_buffer.clear();
                        open_buffer.extend(
                            forward_open
                            .iter()
                            .map(|entry| from_index(entry.0.index, cols)),
                    );
                    open_buffer.extend(
                        backward_open
                            .iter()
                            .map(|entry| from_index(entry.0.index, cols)),
                    );
                    emit_step(
                        &mut on_step,
                        step_index,
                        from_index(current_idx, cols),
                        &visited_coords,
                        &open_buffer,
                        &blocked_cells,
                        true,
                    );
                }
                break Some(current_idx);
            }

            let (r, c) = from_index(current_idx, cols);
            for &(dr, dc) in directions.iter() {
                let nr = r as isize + dr;
                let nc = c as isize + dc;
                if !is_valid(nr, nc, rows, cols) {
                    let candidate = (nr, nc);
                    if blocked_seen.insert(candidate) {
                        blocked_cells.push(candidate);
                    }
                    continue;
                }

                let nr_usize = nr as usize;
                let nc_usize = nc as usize;
                let next_idx = to_index(nr_usize, nc_usize, cols);
                let next = (nr_usize, nc_usize);

                if !is_unblocked(grid, (r, c), next, field_config, robot_config) {
                    let candidate = (nr, nc);
                    if blocked_seen.insert(candidate) {
                        blocked_cells.push(candidate);
                    }
                    continue;
                }
                if visited_backward[next_idx] {
                    continue;
                }

                let base_step = if dr != 0 && dc != 0 { SQRT_2 } else { 1.0 };
                let cost_multiplier = cost_map
                    .map(|cm| cm[(nr_usize, nc_usize)])
                    .unwrap_or(1.0);
                let step_cost = base_step * cost_multiplier;
                let tentative_g = g_backward[current_idx] + step_cost;

                if tentative_g < g_backward[next_idx] {
                    g_backward[next_idx] = tentative_g;
                    parent_backward[next_idx] = current_idx;
                    let f_cost =
                        tentative_g + remaining_path(robot_config, nr_usize, nc_usize, start);
                    backward_open.push(Reverse(QueueNode::new(f_cost, next_idx)));
                }
            }

            if on_step.is_some() {
                open_buffer.clear();
                open_buffer.extend(
                    forward_open
                        .iter()
                        .map(|entry| from_index(entry.0.index, cols)),
                );
                open_buffer.extend(
                    backward_open
                        .iter()
                        .map(|entry| from_index(entry.0.index, cols)),
                );
                emit_step(
                    &mut on_step,
                    step_index,
                    from_index(current_idx, cols),
                    &visited_coords,
                    &open_buffer,
                    &blocked_cells,
                    false,
                );
            }
            step_index += 1;
            break None;
        };

        if let Some(idx) = meeting_from_backward {
            meet_idx = Some(idx);
            break;
        }
    }

    let Some(meeting_idx) = meet_idx else {
        return (None, blocked_cells);
    };

    let mut path: Vec<(usize, usize)> = Vec::new();
    let mut node_idx = meeting_idx;
    while node_idx != start_idx {
        path.push(from_index(node_idx, cols));
        node_idx = parent_forward[node_idx];
    }
    path.push(start);
    path.reverse();

    let mut node_idx = meeting_idx;
    loop {
        let parent_idx = parent_backward[node_idx];
        if parent_idx == usize::MAX || parent_idx == node_idx {
            break;
        }
        node_idx = parent_idx;
        path.push(from_index(node_idx, cols));
    }

    (Some(path), blocked_cells)
}
