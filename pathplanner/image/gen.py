import numpy as np
from PIL import Image, ImageFilter


def noisy_square(width: int, seed: int | None = None, blur_radius: float | None = None) -> Image.Image:
    rng = np.random.default_rng(seed)
    data = rng.integers(0, 256, size=(width, width, 3), dtype=np.uint8)
    img = Image.fromarray(data, mode="RGB")
    if blur_radius is not None and blur_radius > 0:
        img = img.filter(ImageFilter.GaussianBlur(blur_radius))
    return img
