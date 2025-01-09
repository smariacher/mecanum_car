import serial
import time
ser = serial.Serial('COM22', 9600, timeout=1)

while True:
    ser.write(f'00000000000000000000\r'.encode())
    time.sleep(8)
    ser.write(f'35000350003500035000\r'.encode())
    time.sleep(8)