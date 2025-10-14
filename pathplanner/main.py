from image.gen import noisy_square
from image.process import get_red, normalize

img = noisy_square(100)
img.save("output/noisy.png")

red_img = get_red(img)
red_img.save("output/red.png")

n = normalize(red_img)

print(n)
