import pygame
import serial
import time
import RPi.GPIO as GPIO

# Stepper motor GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
PULL1, DIR1 = 5, 6
PULL2, DIR2 = 13, 19
GPIO.setup([PULL1, DIR1, PULL2, DIR2], GPIO.OUT)

# Serial setup
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 9600
TIMEOUT = 1

def stop_steppers():
    print("Stopping steppers")

def gripper_up():
    print("Gripper moving up")
    GPIO.output(DIR1, GPIO.HIGH)
    GPIO.output(DIR2, GPIO.HIGH)
    for _ in range(200):
        GPIO.output(PULL1, GPIO.HIGH)
        GPIO.output(PULL2, GPIO.HIGH)
        time.sleep(0.0008)
        GPIO.output(PULL1, GPIO.LOW)
        GPIO.output(PULL2, GPIO.LOW)
        time.sleep(0.0008)

def gripper_down():
    print("Gripper moving down")
    GPIO.output(DIR1, GPIO.LOW)
    GPIO.output(DIR2, GPIO.LOW)
    for _ in range(200):
        GPIO.output(PULL1, GPIO.HIGH)
        GPIO.output(PULL2, GPIO.HIGH)
        time.sleep(0.0008)
        GPIO.output(PULL1, GPIO.LOW)
        GPIO.output(PULL2, GPIO.LOW)
        time.sleep(0.0008)

def hand_rotate(direction):
    print(f"Hand rotating {direction}")
    GPIO.output(DIR1, GPIO.HIGH if direction == 'right' else GPIO.LOW)
    GPIO.output(DIR2, GPIO.LOW if direction == 'right' else GPIO.HIGH)
    for _ in range(50):
        GPIO.output(PULL1, GPIO.HIGH)
        GPIO.output(PULL2, GPIO.HIGH)
        time.sleep(0.0008)
        GPIO.output(PULL1, GPIO.LOW)
        GPIO.output(PULL2, GPIO.LOW)
        time.sleep(0.0008)

# Establish Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
except Exception as e:
    print(f"Serial init failed: {e}")
    exit()

def establish_connection():
    ser.flushInput()
    ser.flushOutput()
    start_time = time.time()
    while time.time() - start_time < 10:
        ser.write(b"HELLO\n")
        response = ser.read_until(b'\n').decode('ascii', errors='ignore').strip()
        if "ARDUINO_READY" in response:
            print("Connected to Arduino!")
            return True
        time.sleep(0.5)
    return False

if not establish_connection():
    print("Connection failed.")
    ser.close()
    exit()

# Initialize pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Controller: {joystick.get_name()}")

# Joystick mapping
AXIS_X = 0
AXIS_Y = 1
AXIS_Z = 3
BTN_LASER = 0
BTN_OPEN = 0
BTN_CLOSE = 0
BTN_EMG_STOP = 8
BTN_ROTATE_LEFT = 1
BTN_ROTATE_RIGHT = 3
BTN_GRIP_UP = 6
BTN_GRIP_DOWN = 7

DEAD_ZONE = 0.2
SEND_INTERVAL = 0.05
last_send = time.time()

def map_joystick(value):
    return 0 if abs(value) < DEAD_ZONE else (1 if value > 0 else -1)

try:
    while True:
        pygame.event.pump()
        now = time.time()

        x = map_joystick(joystick.get_axis(AXIS_X))
        y = map_joystick(joystick.get_axis(AXIS_Y))
        z = map_joystick(joystick.get_axis(AXIS_Z))

        b = [joystick.get_button(i) for i in range(9)]

        if b[BTN_EMG_STOP]:
            stop_steppers()
            ser.write(b"STOP\n")
            continue

        if b[BTN_ROTATE_LEFT]: hand_rotate('left')
        if b[BTN_ROTATE_RIGHT]: hand_rotate('right')
        if b[BTN_GRIP_UP]: gripper_up()
        if b[BTN_GRIP_DOWN]: gripper_down()

        if (x or y or z or any(b)) and (now - last_send >= SEND_INTERVAL):
            cmd = f"{x},{y},{z},{int(b[0])},{int(b[1])},{int(b[2])},{int(b[3])},{int(b[4])},{int(b[5])},{int(b[6])},{int(b[7])}\n"
            ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
            last_send = now

        time.sleep(0.01)

except KeyboardInterrupt:
    ser.write(b"STOP\n")
    ser.close()
    pygame.quit()
    print("Shutdown complete.")
