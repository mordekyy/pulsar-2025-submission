from image.gen import noisy_square
from image.process import get_red, normalize
from config import IMAGE_SIZE, MAX_SLOPE_DEG, MAX_SLOPE_RAD, BLUR_SIZE

print(f"Generating image with {IMAGE_SIZE}")
img = noisy_square(IMAGE_SIZE, None, BLUR_SIZE)
img.save("output/noisy.png")

red_img = get_red(img)
red_img.save("output/red.png")

n = normalize(red_img)

print(n)
print(f"Max slope is {MAX_SLOPE_DEG}deg")
