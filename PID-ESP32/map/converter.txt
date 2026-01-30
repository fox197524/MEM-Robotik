import sys
from PIL import Image

# --- Configuration ---
IMAGE_FILE = "map.png"  # The name of your input image file
OUTPUT_FILE = "map_image.h"  # The name of the output C header file
ARRAY_NAME = "map_data"      # The name of the C array in the header file
VALUES_PER_LINE = 12         # How many values to put on each line for readability
# ---------------------

def rgb_to_rgb565(r, g, b):
    """Converts a 24-bit RGB color to a 16-bit RGB565 color."""
    r_565 = (r >> 3) & 0x1F
    g_565 = (g >> 2) & 0x3F
    b_565 = (b >> 3) & 0x1F
    return (r_565 << 11) | (g_565 << 5) | b_565

try:
    print(f"Opening image: {IMAGE_FILE}")
    img = Image.open(IMAGE_FILE)
except FileNotFoundError:
    print(f"Error: The file '{IMAGE_FILE}' was not found.")
    sys.exit(1)

# Ensure the image is in RGB format
img = img.convert('RGB')
width, height = img.size
pixels = list(img.getdata())

print(f"Image dimensions: {width}x{height}")
print(f"Total pixels: {len(pixels)}")
print(f"Generating C array '{ARRAY_NAME}'...")

with open(OUTPUT_FILE, "w") as f:
    f.write(f"// Image: {IMAGE_FILE}\n")
    f.write(f"// Dimensions: {width}x{height}\n")
    f.write(f"#include <stdint.h>\n\n")
    f.write(f"const uint16_t {ARRAY_NAME}[{len(pixels)}] = {{\n    ")

    count = 0
    for r, g, b in pixels:
        color16 = rgb_to_rgb565(r, g, b)
        f.write(f"0x{color16:04X}, ")
        count += 1
        if count % VALUES_PER_LINE == 0:
            f.write("\n    ")

    f.write("\n};\n")

print(f"Successfully created '{OUTPUT_FILE}'!")