import numpy as np
from PIL import Image


def noisy_square(width: int, seed: int | None = None) -> Image.Image:
    rng = np.random.default_rng(seed)
    data = rng.integers(0, 256, size=(width, width, 3), dtype=np.uint8)
    return Image.fromarray(data, mode="RGB")
