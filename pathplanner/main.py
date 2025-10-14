from config import MovementMode
from finder.finder import a_star
from finder.messaging import SearchStep
from finder.convert import convert_heightmap_to_meters
from image.gen import noisy_square
from image.process import get_red, normalize
from image.draw import draw_path
from image.animate import render_search_gif
from config import FIELD_CONFIG, ROBOT_CONFIG

print(f"Generating image with {FIELD_CONFIG.IMAGE_SIZE}")
img = noisy_square(FIELD_CONFIG.IMAGE_SIZE, None, FIELD_CONFIG.BLUR_SIZE)
img.save("output/noisy.png")

red_img = get_red(img)
red_img.save("output/red.png")

map_n = normalize(red_img)


print(f"Max slope is {ROBOT_CONFIG.MAX_SLOPE_DEG}deg")


map_m = convert_heightmap_to_meters(map_n, FIELD_CONFIG)

print(map_m)

trace = []
blocked_seen = set()


def record_step(snapshot: SearchStep):
    blocked_cells = tuple(
        cell for cell in snapshot.blocked if cell not in blocked_seen)
    blocked_seen.update(snapshot.blocked)
    trace.append(
        {
            "step": snapshot.step_index,
            "current": snapshot.current,
            "visited": snapshot.visited,
            "open": snapshot.open_nodes,
            "blocked": blocked_cells,
        }
    )


p, b = a_star(map_m, (0, 0), (99, 99),
              MovementMode.FOUR_DIRECTIONS, on_step=record_step)
print(p)
print(b)

p_map = draw_path(red_img, p, b)
p_map.save("output/path.png")

render_search_gif(red_img, trace, p, "output/search.gif")
