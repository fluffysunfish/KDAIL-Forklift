import RPi.GPIO as GPIO
import sys
import tty
import termios
import time

# Define GPIO pin connections
IN1 = 18  # Left motor forward
IN2 = 17  # Left motor backward
IN3 = 22  # Right motor forward
IN4 = 23  # Right motor backward
EXTRA_PIN = 24  # Additional GPIO for high-current control (e.g., relay)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(EXTRA_PIN, GPIO.OUT)  # Setup the extra GPIO pin

def getch():
    """Get a single character from standard input"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def stop_motors():
    print("Stopping motors")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def forward():
    print("Moving forward")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def backward():
    print("Moving backward")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def turn_left():
    print("Turning left")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_right():
    print("Turning right")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def extra_pin_high():
    print("Extra pin (GPIO 24) set to HIGH")
    GPIO.output(EXTRA_PIN, GPIO.HIGH)

def extra_pin_low():
    print("Extra pin (GPIO 24) set to LOW")
    GPIO.output(EXTRA_PIN, GPIO.LOW)

try:
    print("WASD Motor Control with Extra GPIO")
    print("---------------------------------")
    print("W: Forward")
    print("S: Backward")
    print("A: Turn Left")
    print("D: Turn Right")
    print("Space: Stop Motors")
    print("H: Set Extra Pin (GPIO 24) HIGH")
    print("L: Set Extra Pin (GPIO 24) LOW")
    print("Q: Quit")

    while True:
        char = getch().lower()

        if char == 'w':
            forward()
        elif char == 's':
            backward()
        elif char == 'a':
            turn_left()
        elif char == 'd':
            turn_right()
        elif char == ' ':
            stop_motors()
        elif char == 'h':
            extra_pin_high()
        elif char == 'l':
            extra_pin_low()
        elif char == 'q':
            break

except KeyboardInterrupt:
    print("\nProgram terminated by user")
except Exception as e:
    print(f"\nAn error occurred: {e}")
finally:
    stop_motors()
    GPIO.output(EXTRA_PIN, GPIO.LOW)  # Ensure extra pin is LOW on exit
    GPIO.cleanup()
    print("GPIO pins cleaned up")
