import pygame as pg
import sys

# Initialize pg
pg.init()

# Initialize the joystick module
pg.joystick.init()

# Check for joystick availability
if pg.joystick.get_count() == 0:
    print("No joystick connected")
    sys.exit()

# Initialize the first joystick
joystick = pg.joystick.Joystick(0)
joystick.init()

print(f"Joystick name: {joystick.get_name()}")

# Set up the display
screen_width, screen_height = 800, 800
screen = pg.display.set_mode((screen_width, screen_height))
pg.display.set_caption("Joystick Axis Display")

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
DARK_GREY = (100,100,100)

# Function to draw the joystick axes
def draw_axes(screen, joystick):
    num_axes = joystick.get_numaxes()
    for i in range(2):
        axis_value = joystick.get_axis(i)
        bar_length = (axis_value + 1) * (screen_width / 4)
        bar_x = screen_width / 4
        bar_y = 50 + i * 50
        pg.draw.rect(screen, RED, [bar_x, bar_y, bar_length, 20])
        pg.draw.rect(screen, BLACK, [bar_x, bar_y, screen_width / 2, 20], 2)
        axis_label = f"Axis {i}: {axis_value:.2f}"
        font = pg.font.Font(None, 36)
        text = font.render(axis_label, True, WHITE)
        screen.blit(text, (bar_x + screen_width / 2 + 10, bar_y))

def draw_wheels(width, height, offset):
    

    pg.draw.rect(screen, DARK_GREY, [offset, offset, width, height])
    pg.draw.rect(screen, DARK_GREY, [screen_height - offset - width, offset, width, height])
    pg.draw.rect(screen, DARK_GREY, [offset, screen_height - offset - height, width, height])
    pg.draw.rect(screen, DARK_GREY, [screen_height - offset - width, screen_height - offset - height, width, height])

def draw_arrow(screen, start_pos, end_pos, color):
    arrow_width = 10
    arrow_head_length = 20

    # Calculate the direction vector
    direction = pg.math.Vector2(end_pos) - pg.math.Vector2(start_pos)
    length = direction.length()
    if length == 0:
        return

    direction = direction.normalize()

    # Calculate the points for the arrow head
    left = pg.math.Vector2(-direction.y, direction.x) * arrow_width
    right = pg.math.Vector2(direction.y, -direction.x) * arrow_width
    arrow_tip = pg.math.Vector2(end_pos)
    arrow_base = arrow_tip - direction * arrow_head_length

    # Draw the arrow line
    pg.draw.line(screen, color, start_pos, arrow_base, 2)

    # Draw the arrow head
    points = [arrow_tip, arrow_base + left, arrow_base + right]
    pg.draw.polygon(screen, color, points)    

def draw_speed_arrows(offset_x, offset_y, wheel_speed):
    multiplier = 0.5
    for i in range(4):
        wheel_speeds[i] *= multiplier

    draw_arrow(screen, [offset_x, offset_y], [offset_x, offset_y + wheel_speed[0]], RED)
    draw_arrow(screen, [screen_width - offset_x, offset_y], [screen_width - offset_x, offset_y + wheel_speed[1]], RED)
    draw_arrow(screen, [offset_x, screen_height - offset_y], [offset_x, screen_height - offset_y + wheel_speed[2]], RED)
    draw_arrow(screen, [screen_width - offset_x, screen_height - offset_y], [screen_width - offset_x, screen_height - offset_y + wheel_speed[3]], RED)
    

def get_wheel_speeds(joystick):
    speeds = [0,0,0,0]
    x = joystick.get_axis(0)*255
    y = joystick.get_axis(1)*255

    clamp = 20

    if x < clamp and x > -clamp:
        x = 0
    if y < clamp and y > -clamp:
        y = 0


    speeds[0] = y + x
    speeds[1] = y - x
    speeds[2] = y + x
    speeds[3] = y - x

    return speeds

def draw_direction(joystick):
    multiplier = 100
    x = joystick.get_axis(0)*multiplier
    y = joystick.get_axis(1)*multiplier

    draw_arrow(screen, [screen_width/2, screen_height/2], [screen_width/2 + x, screen_height/2 + y], RED)

try:
    while True:
        wheel_width = 120
        wheel_height = 200
        offset = 100

        # Handle events
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()

        # Clear the screen
        screen.fill(BLACK)

        # Draw the joystick axes
        # draw_axes(screen, joystick)
        draw_wheels(wheel_width, wheel_height, offset)
        wheel_speeds = get_wheel_speeds(joystick)
        draw_speed_arrows(offset + (wheel_width/2), offset + (wheel_height/2), wheel_speeds)
        draw_direction(joystick)
        
        # Update the display
        pg.display.flip()

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    # Quit pg
    pg.quit()
