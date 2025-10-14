from PIL import Image, ImageDraw


def draw_path(img: Image.Image, path: list[tuple[int, int]], blocked: list[tuple[int, int]]) -> Image.Image:
    out = img.convert("RGB") if img.mode not in {"RGB", "RGBA"} else img.copy()
    draw = ImageDraw.Draw(out)

    for (r, c) in blocked:
        draw.point((c, r), fill="black")

    for (r, c) in path or []:
        draw.point((c, r), fill="red")

    if path:
        sr, sc = path[0]
        draw.point((sc, sr), fill="green")

        er, ec = path[-1]
        draw.point((ec, er), fill="purple")

    return out
