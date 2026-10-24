"""Floor colours from a downward camera, including shaded tiles."""

import math


def read_floor_rgb(camera):
    """Average the central 7x7 pixels; return None until an image exists."""

    image = camera.getImage()

    if image is None:
        return None

    width = camera.getWidth()
    height = camera.getHeight()

    # Sample a 7x7 square at the centre of the image.
    center_x = width // 2
    center_y = height // 2
    radius = 3

    red_total = 0
    green_total = 0
    blue_total = 0
    count = 0

    for y in range(max(0, center_y - radius),
                   min(height, center_y + radius + 1)):

        for x in range(max(0, center_x - radius),
                       min(width, center_x + radius + 1)):

            red_total += camera.imageGetRed(image, width, x, y)
            green_total += camera.imageGetGreen(image, width, x, y)
            blue_total += camera.imageGetBlue(image, width, x, y)

            count += 1

    if count == 0:
        return None

    return (red_total / count, green_total / count, blue_total / count)


def classify_floor_rgb(rgb):
    """Use channel dominance before brightness to preserve dark colours.

    Under-chassis shadows can reduce a green/blue tile below the former
    absolute 60/255 cutoff. Saturated colours remain distinguishable from
    the neutral black floor at these brightness levels. Near-black noise,
    weak colour casts, and orange chassis pixels are not tile evidence.
    """
    if rgb is None:
        return "unknown"

    r, g, b = rgb
    if not all(math.isfinite(v) and 0 <= v <= 255 for v in rgb):
        return "unknown"

    brightest = max(rgb)
    darkest = min(rgb)
    chromatic = brightest >= 20 and brightest - darkest >= 12

    # Require red to be well separated from orange/yellow as well as blue.
    if chromatic and r > 2.2 * g and r > 1.5 * b:
        return "red"
    if chromatic and g > 1.5 * r and g > 1.5 * b:
        return "green"
    if chromatic and b > 1.5 * r and b > 1.5 * g:
        return "blue"

    # Detect black.
    if brightest < 50 and brightest - darkest < 12:
        return "black"

    # Detect white.
    if darkest > 170 and brightest - darkest < 50:
        return "white"

    return "unknown"


def read_floor(camera):
    """Return red, green, blue, black, white, or unknown."""
    return classify_floor_rgb(read_floor_rgb(camera))
