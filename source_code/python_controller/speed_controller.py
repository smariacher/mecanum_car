import pygame
import serial
import time

# Initialize Pygame and the controller
pygame.init()
pygame.joystick.init()

# Check for available controllers
if pygame.joystick.get_count() == 0:
    print("No controller found.")
    exit()

# Initialize the first connected controller
controller = pygame.joystick.Joystick(0)
controller.init()

# Initialize serial communication (adjust COM port and baud rate)
ser = serial.Serial('COM9', 9600, timeout=1)  # Adjust the port and baudrate accordingly
time.sleep(2)  # Wait for the serial connection to initialize

try:
    while True:
        pygame.event.pump()  # Process event queue

        # Read the left trigger (usually axis 2)
        left_trigger_value = controller.get_axis(2)  # Change index if different for your controller
        # Map the value from -1..1 to 0..255 (if the controller reports negative values)
        left_trigger_value = int((left_trigger_value + 1) / 2 * 255)

        # Read the right trigger (usually axis 5)
        right_trigger_value = controller.get_axis(5)  # Change index if different for your controller
        # Map the value from -1..1 to 0..255 (if the controller reports negative values)
        right_trigger_value = int((right_trigger_value + 1) / 2 * 10000)

        # Send the trigger values via serial
        ser.write(f"{right_trigger_value}\r".encode())
        print(f"{right_trigger_value}\r")

        time.sleep(0.1)  # Adjust the sleep time for better responsiveness

except KeyboardInterrupt:
    print("Exiting program.")

finally:
    ser.close()
    pygame.quit()
