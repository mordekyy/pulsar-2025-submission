from config import MovementMode
from finder.finder import a_star
from finder.convert import convert_heightmap_to_meters
from image.gen import noisy_square
from image.process import get_red, normalize
from image.draw import draw_path
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

p = a_star(map_m, (0, 0), (99, 99), MovementMode.FOUR_DIRECTIONS)
print(p)

p_map = draw_path(red_img, p)
p_map.save("output/path.png")
