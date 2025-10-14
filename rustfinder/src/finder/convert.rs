use ndarray::Array2;

use crate::config::{FieldConfig, RobotConfig};

use super::Grid;

pub fn convert_heightmap_to_meters(map: &Grid, field_config: &FieldConfig) -> Grid {
    let scale = field_config.height_max_m - field_config.height_min_m;
    map.map(|v| field_config.height_min_m + *v * scale)
}

fn gradient_components(height_map: &Grid, pixel_size: f32) -> (Grid, Grid) {
    let rows = height_map.nrows();
    let cols = height_map.ncols();
    let mut dx = Array2::<f32>::zeros((rows, cols));
    let mut dy = Array2::<f32>::zeros((rows, cols));

    let denom = 2.0 * pixel_size;

    for r in 0..rows {
        for c in 0..cols {
            let left = if c == 0 {
                height_map[(r, c)]
            } else {
                height_map[(r, c - 1)]
            };
            let right = if c + 1 >= cols {
                height_map[(r, c)]
            } else {
                height_map[(r, c + 1)]
            };
            dx[(r, c)] = (right - left) / denom;

            let up = if r == 0 {
                height_map[(r, c)]
            } else {
                height_map[(r - 1, c)]
            };
            let down = if r + 1 >= rows {
                height_map[(r, c)]
            } else {
                height_map[(r + 1, c)]
            };
            dy[(r, c)] = (down - up) / denom;
        }
    }

    (dx, dy)
}

fn clip_unit(val: f32) -> f32 {
    if val < 0.0 {
        0.0
    } else if val > 1.0 {
        1.0
    } else {
        val
    }
}

pub fn compute_gradient_cost_map(
    height_map: &Grid,
    field_config: &FieldConfig,
    robot_config: &RobotConfig,
) -> Grid {
    let (dx, dy) = gradient_components(height_map, field_config.pixel_size_m);
    let mut out = Array2::<f32>::zeros(height_map.dim());

    let max_slope = robot_config.max_slope_deg;
    if max_slope <= 0.0 {
        out.fill(1.0);
        return out;
    }

    for ((r, c), value) in out.indexed_iter_mut() {
        let slope = dx[(r, c)].hypot(dy[(r, c)]);
        let slope_angle = slope.atan().to_degrees();
        let ratio = clip_unit(slope_angle / max_slope);
        *value = 1.0 + robot_config.slope_cost_weight * ratio * ratio;
    }

    out
}
