use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::f32::consts::SQRT_2;

use ndarray::Array2;
use ordered_float::NotNan;

use crate::config::{FieldConfig, MovementMode, RobotConfig};

use super::messaging::{emit_step, SearchStepCallback};

pub type Grid = Array2<f32>;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
struct QueueNode {
    f_cost: NotNan<f32>,
    position: (usize, usize),
}

impl QueueNode {
    fn new(f_cost: f32, position: (usize, usize)) -> Self {
        let f = NotNan::new(f_cost).unwrap_or_else(|_| NotNan::new(f32::INFINITY).unwrap());
        Self { f_cost: f, position }
    }
}

impl Ord for QueueNode {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.f_cost.cmp(&other.f_cost) {
            Ordering::Equal => self.position.cmp(&other.position),
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

fn remaining_path(row: usize, col: usize, dest: (usize, usize)) -> f32 {
    let (dr, dc) = (
        row as f32 - dest.0 as f32,
        col as f32 - dest.1 as f32,
    );
    dr.hypot(dc)
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

    let mut blocked_cells: Vec<(isize, isize)> = Vec::new();

    if !is_valid(start.0 as isize, start.1 as isize, rows, cols) {
        return (None, blocked_cells);
    }
    if !is_valid(end.0 as isize, end.1 as isize, rows, cols) {
        return (None, blocked_cells);
    }
    if start == end {
        return (Some(vec![start]), blocked_cells);
    }

    let mut open_queue: BinaryHeap<Reverse<QueueNode>> = BinaryHeap::new();
    let mut g_score: HashMap<(usize, usize), f32> = HashMap::new();
    let mut parent: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
    let mut visited: HashSet<(usize, usize)> = HashSet::new();

    g_score.insert(start, 0.0);
    parent.insert(start, start);
    let f0 = remaining_path(start.0, start.1, end);
    open_queue.push(Reverse(QueueNode::new(f0, start)));

    let mut step_index = 0_usize;
    let directions = movement_mode.directions();

    while let Some(Reverse(node)) = open_queue.pop() {
        let current = node.position;
        if visited.contains(&current) {
            continue;
        }

        if current == end {
            let mut snapshot_iter = visited.iter().copied().collect::<Vec<_>>();
            snapshot_iter.push(current);
            let open_snapshot: Vec<(usize, usize)> =
                open_queue.iter().map(|entry| entry.0.position).collect();
            emit_step(
                &mut on_step,
                step_index,
                current,
                snapshot_iter.into_iter(),
                &open_snapshot,
                &blocked_cells,
            );

            let mut path = Vec::new();
            let mut node_pos = current;
            while node_pos != start {
                path.push(node_pos);
                node_pos = parent[&node_pos];
            }
            path.push(start);
            path.reverse();
            return (Some(path), blocked_cells);
        }

        visited.insert(current);

        let (r, c) = current;
        for &(dr, dc) in directions.iter() {
            let nr = r as isize + dr;
            let nc = c as isize + dc;
            if !is_valid(nr, nc, rows, cols) {
                blocked_cells.push((nr, nc));
                continue;
            }

            let nr_usize = nr as usize;
            let nc_usize = nc as usize;
            let next = (nr_usize, nc_usize);

            if !is_unblocked(grid, (r, c), next, field_config, robot_config) {
                blocked_cells.push((nr, nc));
                continue;
            }
            if visited.contains(&next) {
                continue;
            }

            let base_step = if dr != 0 && dc != 0 { SQRT_2 } else { 1.0 };
            let cost_multiplier = cost_map
                .map(|cm| cm[(nr_usize, nc_usize)])
                .unwrap_or(1.0);
            let step_cost = base_step * cost_multiplier;
            let tentative_g = g_score.get(&current).copied().unwrap_or(f32::INFINITY) + step_cost;

            if tentative_g < g_score.get(&next).copied().unwrap_or(f32::INFINITY) {
                g_score.insert(next, tentative_g);
                parent.insert(next, current);
                let f_cost = tentative_g + remaining_path(nr_usize, nc_usize, end);
                open_queue.push(Reverse(QueueNode::new(f_cost, next)));
            }
        }

        let visited_iter = visited.iter().copied();
        let open_snapshot: Vec<(usize, usize)> =
            open_queue.iter().map(|entry| entry.0.position).collect();
        emit_step(
            &mut on_step,
            step_index,
            current,
            visited_iter,
            &open_snapshot,
            &blocked_cells,
        );
        step_index += 1;
    }

    (None, blocked_cells)
}
