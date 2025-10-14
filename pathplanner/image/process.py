from PIL import Image


def get_red(img: Image.Image) -> Image.Image:
    r, g, b = img.split()
    return r
