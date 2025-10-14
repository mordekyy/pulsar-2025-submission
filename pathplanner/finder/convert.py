from config import FieldConfig, MovementMode
from numpy import ndarray


def convert_heightmap_to_meters(map: ndarray, field_config: FieldConfig) -> ndarray:
    return field_config.HEIGHT_MIN_M + map * (field_config.HEIGHT_MAX_M - field_config.HEIGHT_MIN_M)


def directions_from_movement_mode(mode: MovementMode):
    if mode == MovementMode.FOUR_DIRECTIONS:
        return [(1, 0), (0, 1), (-1, 0), (0, -1)]
    elif mode == MovementMode.EIGHT_DIRECTIONS:
        return [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    return []
