import pygame as pg
import sys
import serial
import time
import math

# VISUAL_MODE opens a window for speed debugging
VISUAL_MODE = False 

# Initialize pg
pg.init()

# Set up display
screen_width, screen_height = 300, 300
screen = pg.display.set_mode((screen_width, screen_height))
pg.display.set_caption("Robot Controller")

# Initialize the joystick module
pg.joystick.init()

# Check for joystick availability
joystick = None
if pg.joystick.get_count() > 0:
    joystick = pg.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick name: {joystick.get_name()}")
else:
    print("No joystick connected. Using keyboard input.")

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
DARK_GREY = (100,100,100)

# Keyboard input handling function
def get_keyboard_input():
    x, y, turn = 0, 0, 0

    # Movement controls
    if pg.key.get_pressed()[pg.K_w]:
        y -= 1
    if pg.key.get_pressed()[pg.K_s]:
        y += 1
    if pg.key.get_pressed()[pg.K_a]:
        x -= 1
    if pg.key.get_pressed()[pg.K_d]:
        x += 1

    # Turning controls
    if pg.key.get_pressed()[pg.K_q]:
        turn -= 1
    if pg.key.get_pressed()[pg.K_e]:
        turn += 1

    return x, y, turn

# Function to draw the joystick axes
def draw_axes(screen, joystick):
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
        wheel_speed[i] *= multiplier

    draw_arrow(screen, [offset_x, offset_y], [offset_x, offset_y + wheel_speed[0]], RED)
    draw_arrow(screen, [screen_width - offset_x, offset_y], [screen_width - offset_x, offset_y + wheel_speed[1]], RED)
    draw_arrow(screen, [offset_x, screen_height - offset_y], [offset_x, screen_height - offset_y + wheel_speed[2]], RED)
    draw_arrow(screen, [screen_width - offset_x, screen_height - offset_y], [screen_width - offset_x, screen_height - offset_y + wheel_speed[3]], RED)

def draw_direction(input_source, is_joystick=True):
    multiplier = 100
    if is_joystick:
        x = -input_source.get_axis(0)*multiplier
        y = -input_source.get_axis(1)*multiplier
    else:
        x, y, _ = input_source
        x *= multiplier
        y *= multiplier

    draw_arrow(screen, [screen_width/2, screen_height/2], [screen_width/2 + x, screen_height/2 + y], RED)

def get_wheel_speeds(input_source, is_joystick=True):
    speeds = [0,0,0,0]
    
    if is_joystick:
        x = -input_source.get_axis(0)*255
        y = -input_source.get_axis(1)*255
        turn = input_source.get_axis(2)*255
    else:
        x, y, turn = input_source
        x *= 300
        y *= 300
        turn *= 300

    clamp_x = 20
    clamp_y = 20
    clamp_turn = 20

    if turn < clamp_turn and turn > -clamp_turn:
        turn  = 0

    if x < clamp_x and x > -clamp_x:
        x = 0
    if y < clamp_y and y > -clamp_y:
        y = 0

    theta = math.atan2(y, x)
    power = math.hypot(x, y)

    sin = math.sin(theta - math.pi/4)
    cos = math.cos(theta - math.pi/4)
    this_max = max(abs(sin), abs(cos))

    if this_max == 0:
        this_max = 0.1

    speeds[0] = power * cos/this_max + turn
    speeds[1] = power * sin/this_max - turn
    speeds[2] = power * sin/this_max + turn
    speeds[3] = power * cos/this_max - turn

    return speeds

if VISUAL_MODE:
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

            # Draw axes and speed based on input source
            if joystick:
                draw_axes(screen, joystick)
                wheel_speeds = get_wheel_speeds(joystick)
                draw_direction(joystick)
            else:
                keyboard_input = get_keyboard_input()
                wheel_speeds = get_wheel_speeds(keyboard_input, is_joystick=False)
                draw_direction(keyboard_input, is_joystick=False)

            draw_wheels(wheel_width, wheel_height, offset)
            draw_speed_arrows(offset + (wheel_width/2), offset + (wheel_height/2), wheel_speeds)

            # Update the display
            pg.display.flip()
            pg.time.delay(50)  # Add a small delay to prevent high CPU usage

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        # Quit pg
        pg.quit()

else:
    ser = serial.Serial('COM22',
                     9600,
                     bytesize=serial.EIGHTBITS,
                     stopbits=serial.STOPBITS_ONE,
                     parity=serial.PARITY_NONE,
                     xonxoff=True,
                     timeout=1
                    )
    time.sleep(2)
    try:
        while True:
            
            # Handle events
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit()

            # Get wheel speeds from joystick or keyboard
            if joystick:
                wheel_speeds = get_wheel_speeds(joystick)
                
                if joystick.get_button(0):
                    ser.write('e\r'.encode())
                
                if joystick.get_button(1):
                    ser.write('f\r'.encode())
                    print("toggled front light!")
                    time.sleep(0.1)
                
                if joystick.get_button(2):
                    ser.write('b\r'.encode())
                    print("toggled back light!")
                    time.sleep(0.1)
            else:
                keyboard_input = get_keyboard_input()
                wheel_speeds = get_wheel_speeds(keyboard_input, is_joystick=False)
                
                # Keyboard button mappings
                keys = pg.key.get_pressed()
                if keys[pg.K_SPACE]:
                    ser.write('e\r'.encode())
                
                if keys[pg.K_f]:
                    ser.write('f\r'.encode())
                    print("toggled front light!")
                    time.sleep(0.1)

                if keys[pg.K_b]:
                    ser.write('b\r'.encode())
                    print("toggled back light!")
                    time.sleep(0.1)


            multiplier = 20
            send_speed1 = int(wheel_speeds[0]*multiplier)
            send_speed2 = int(wheel_speeds[1]*multiplier)
            send_speed3 = int(wheel_speeds[2]*multiplier)
            send_speed4 = int(wheel_speeds[3]*multiplier)

            ser.write(f'{send_speed1:05}{send_speed2:05}{send_speed3:05}{send_speed4:05}\r'.encode())
            print(f"{send_speed1:05}{send_speed2:05}{send_speed3:05}{send_speed4:05}")
            time.sleep(.1)

    except KeyboardInterrupt:
        ser.write(f'00000000000000000000\r'.encode())
        print("\nExiting...")

    finally:
        # Quit pg
        ser.write(f'00000000000000000000\r'.encode())
        pg.quit()