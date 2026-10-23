
def read_floor(camera):
    """
    Detect the floor colour using the robot's camera.

    Returns:
        "red", "green", "blue",
        "black", "white", or "unknown"
    """

    image = camera.getImage()

    if image is None:
        return "unknown"

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
        return "unknown"

    # Calculate average RGB values.
    r = red_total / count
    g = green_total / count
    b = blue_total / count

    # Detect black.
    if max(r, g, b) < 50:
        return "black"

    # Detect white.
    if min(r, g, b) > 170 and max(r, g, b) - min(r, g, b) < 50:
        return "white"

    # Detect red.
    if r > 70 and r > g * 1.5 and r > b * 1.5:
        return "red"

    # Detect green.
    if g > 60 and g > r * 1.3 and g > b * 1.3:
        return "green"

    # Detect blue.
    if b > 60 and b > r * 1.3 and b > g * 1.3:
        return "blue"

    return "unknown"
