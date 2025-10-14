#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MovementMode {
    FourDirections,
    EightDirections,
}

impl MovementMode {
    pub fn directions(self) -> &'static [(isize, isize)] {
        const FOUR_DIRS: [(isize, isize); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];
        const EIGHT_DIRS: [(isize, isize); 8] = [
            (1, 0),
            (0, 1),
            (-1, 0),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ];

        match self {
            MovementMode::FourDirections => &FOUR_DIRS,
            MovementMode::EightDirections => &EIGHT_DIRS,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FieldConfig {
    pub image_size: u32,
    pub blur_size: f32,
    pub height_min_m: f32,
    pub height_max_m: f32,
    pub pixel_size_m: f32,
}

impl Default for FieldConfig {
    fn default() -> Self {
        Self {
            image_size: 100,
            blur_size: (100.0 * 2.0) / 100.0,
            height_min_m: 0.0,
            height_max_m: 3.0,
            pixel_size_m: 0.1,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RobotConfig {
    pub max_slope_deg: f32,
    pub start_pos: (usize, usize),
    pub movement_mode: MovementMode,
    pub slope_cost_weight: f32,
    pub remaining_distance_weight: f32,
    pub trace_sample_stride: usize,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            max_slope_deg: 30.0,
            start_pos: (0, 0),
            movement_mode: MovementMode::EightDirections,
            slope_cost_weight: 100.0,
            remaining_distance_weight: 10.0,
            trace_sample_stride: 50,
        }
    }
}

pub static FIELD_CONFIG: FieldConfig = FieldConfig {
    image_size: 512,
    blur_size: (100.0 * 3.0) / 100.0,
    height_min_m: 0.0,
    height_max_m: 3.0,
    pixel_size_m: 0.1,
};

pub static ROBOT_CONFIG: RobotConfig = RobotConfig {
    max_slope_deg: 30.0,
    start_pos: (0, 0),
    movement_mode: MovementMode::EightDirections,
    slope_cost_weight: 100.0,
    remaining_distance_weight: 5.0,
    trace_sample_stride: 50,
};
