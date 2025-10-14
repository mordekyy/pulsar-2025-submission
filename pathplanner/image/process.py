from PIL import Image
from numpy import asarray, ndarray, float32


def get_red(img: Image.Image) -> Image.Image:
    r, g, b = img.split()
    return r


def normalize(img: Image.Image) -> ndarray:
    arr = asarray(img, dtype=float32)
    return arr / 255.0
