import sys
from PIL import Image

# --- Configuration ---
IMAGE_FILE = "Ahmete_girecek.png"      # The name of your input image file
OUTPUT_FILE = "MAP.BIN"     # The name of the output binary file (use 8.3 format for SD cards)
# ---------------------

def rgb_to_rgb565_big_endian(r, g, b):
    """Converts a 24-bit RGB color to a 16-bit RGB565 color in Big-Endian format."""
    r_565 = (r >> 3) & 0x1F
    g_565 = (g >> 2) & 0x3F
    b_565 = (b >> 3) & 0x1F
    # Most TFT displays expect Big-Endian format (MSB first)
    color16 = (r_565 << 11) | (g_565 << 5) | b_565
    return color16.to_bytes(2, 'big')

try:
    print(f"Opening image: {IMAGE_FILE}")
    img = Image.open(IMAGE_FILE)
except FileNotFoundError:
    print(f"Error: The file '{IMAGE_FILE}' was not found.")
    sys.exit(1)

img = img.convert('RGB')
width, height = img.size
pixels = list(img.getdata())

print(f"Image dimensions: {width}x{height}")
print(f"Generating binary file '{OUTPUT_FILE}'...")

with open(OUTPUT_FILE, "wb") as f: # 'wb' for write binary
    for r, g, b in pixels:
        pixel_bytes = rgb_to_rgb565_big_endian(r, g, b)
        f.write(pixel_bytes)

print(f"Successfully created '{OUTPUT_FILE}'!")