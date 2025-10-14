from PIL import Image
import numpy as np


def get_red(img: Image.Image) -> Image.Image:
    r, g, b = img.split()
    return r


def normalize(img: Image.Image) -> np.ndarray:
    arr = np.asarray(img, dtype=np.float32)
    return arr / 255.0
