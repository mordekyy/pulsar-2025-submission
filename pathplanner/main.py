from image.gen import noisy_square
from image.process import get_red

img = noisy_square(100)
img.save("noisy.png")

red_img = get_red(img)
red_img.save("red.png")
