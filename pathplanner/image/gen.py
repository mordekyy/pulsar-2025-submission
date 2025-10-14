from PIL import Image, ImageFilter
from numpy import random, uint8


def noisy_square(width: int, seed: int | None = None, blur_radius: float | None = None) -> Image.Image:
    rng = random.default_rng(seed)
    data = rng.integers(0, 256, size=(width, width, 3), dtype=uint8)
    img = Image.fromarray(data, mode="RGB")
    if blur_radius is not None and blur_radius > 0:
        img = img.filter(ImageFilter.GaussianBlur(blur_radius))
    return img
