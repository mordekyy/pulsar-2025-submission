from config import FieldConfig, MovementMode, RobotConfig
from numpy import ndarray, gradient, hypot, clip, arctan
from numpy import degrees


def convert_heightmap_to_meters(map: ndarray, field_config: FieldConfig) -> ndarray:
    return field_config.HEIGHT_MIN_M + map * (field_config.HEIGHT_MAX_M - field_config.HEIGHT_MIN_M)


def compute_gradient_cost_map(
    height_map: ndarray, field_config: FieldConfig, robot_config: RobotConfig
) -> ndarray:
    dx, dy = gradient(height_map, field_config.PIXEL_SIZE_M)
    slope = hypot(dx, dy)
    slope_angle = degrees(arctan(slope))
    max_slope = robot_config.MAX_SLOPE_DEG
    if max_slope <= 0:
        return clip(slope_angle, 0.0, None) * 0.0 + 1.0
    slope_ratio = clip(slope_angle / max_slope, 0.0, 1.0)
    return 1.0 + robot_config.SLOPE_COST_WEIGHT * slope_ratio * slope_ratio


def directions_from_movement_mode(mode: MovementMode):
    if mode == MovementMode.FOUR_DIRECTIONS:
        return [(1, 0), (0, 1), (-1, 0), (0, -1)]
    elif mode == MovementMode.EIGHT_DIRECTIONS:
        return [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    return []
