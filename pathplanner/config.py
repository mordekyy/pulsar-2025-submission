from enum import Enum
from dataclasses import dataclass


class MovementMode(Enum):
    FOUR_DIRECTIONS = 4
    EIGHT_DIRECTIONS = 8


@dataclass(frozen=True)
class FieldConfig:
    IMAGE_SIZE: int = 100
    BLUR_SIZE: float = IMAGE_SIZE * 2 / 100
    HEIGHT_MIN_M: float = 0.0
    HEIGHT_MAX_M: float = 3.0
    PIXEL_SIZE_M: float = 0.1


@dataclass(frozen=True)
class RobotConfig:
    MAX_SLOPE_DEG = 30.0
    START_POS = (0, 0)
    MOVEMENT_MODE = MovementMode.EIGHT_DIRECTIONS
    SLOPE_COST_WEIGHT = 10.0
    REMAINING_DISTANCE_WEIGHT = 10.0
    TRACE_SAMPLE_STRIDE: int = 50


FIELD_CONFIG = FieldConfig()
ROBOT_CONFIG = RobotConfig()
