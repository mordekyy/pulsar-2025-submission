from time import perf_counter

from config import MovementMode
from finder.finder import a_star
from finder.messaging import SearchStep
from finder.convert import convert_heightmap_to_meters, compute_gradient_cost_map
from image.gen import noisy_square
from image.process import get_red, normalize
from image.draw import draw_path
from image.animate import render_search_gif
from config import FIELD_CONFIG, ROBOT_CONFIG

print(f"Generating image with {FIELD_CONFIG.IMAGE_SIZE}")
pipeline_start = perf_counter()
img = noisy_square(FIELD_CONFIG.IMAGE_SIZE, None, FIELD_CONFIG.BLUR_SIZE)
img.save("output/noisy.png")

red_img = get_red(img)
red_img.save("output/red.png")

map_n = normalize(red_img)


print(f"Max slope is {ROBOT_CONFIG.MAX_SLOPE_DEG}deg")


map_m = convert_heightmap_to_meters(map_n, FIELD_CONFIG)
cost_map = compute_gradient_cost_map(map_m, FIELD_CONFIG, ROBOT_CONFIG)

print(map_m)

trace = []
blocked_seen = set()
SAMPLE_STRIDE = ROBOT_CONFIG.TRACE_SAMPLE_STRIDE
DESTINATION = (FIELD_CONFIG.IMAGE_SIZE-1, FIELD_CONFIG.IMAGE_SIZE-1)


def record_step(snapshot: SearchStep):
    if (
        snapshot.step_index % SAMPLE_STRIDE != 0
        and snapshot.current != DESTINATION
        and not snapshot.is_goal
    ):
        return

    blocked_cells = tuple(
        cell for cell in snapshot.blocked if cell not in blocked_seen)
    blocked_seen.update(snapshot.blocked)
    if snapshot.step_index % (SAMPLE_STRIDE * 10) == 0:
        print(
            f"{snapshot.step_index} steps processed (sampling every {SAMPLE_STRIDE})"
        )
    trace.append(
        {
            "step": snapshot.step_index,
            "current": snapshot.current,
            "visited": snapshot.visited,
            "open": snapshot.open_nodes,
            "blocked": blocked_cells,
            "goal": snapshot.is_goal,
        }
    )


search_start = perf_counter()
p, b = a_star(
    map_m,
    (0, 0),
    DESTINATION,
    ROBOT_CONFIG.MOVEMENT_MODE,
    cost_map=cost_map,
    on_step=record_step,
)
search_elapsed = perf_counter() - search_start
print(p)
print(b)

p_map = draw_path(red_img, p, b)
p_map.save("output/path.png")

render_search_gif(red_img, trace, p, "output/search.gif")
pipeline_elapsed = perf_counter() - pipeline_start
print(f"Search time: {search_elapsed:.3f}s")
print(f"Total pipeline time: {pipeline_elapsed:.3f}s")
