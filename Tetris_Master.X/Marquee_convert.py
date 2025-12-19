from PIL import Image

img = Image.open("title.png").convert("1")  # black & white
w, h = img.size
pixels = img.load()

print(f"const bool image[{h}][{w}] = {{")

for y in range(h):
    print("  {", end="")
    for x in range(w):
        # In PIL '1' mode: 0 = black, 255 = white
        val = "true" if pixels[x, y] == 0 else "false"
        print(val, end="")
        if x < w - 1:
            print(", ", end="")
    print("},")
print("};")
