import RPi.GPIO as GPIO
import time

# Define GPIO pin connections
IN1 = 18  # Left motor forward
IN2 = 17  # Left motor backward
IN3 = 22  # Right motor forward
IN4 = 23  # Right motor backward

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def stop_motors():
    print("Stopping motors")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def test_left_motor_forward():
    print("Testing left motor forward...")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(3)
    stop_motors()
    time.sleep(1)

def test_left_motor_backward():
    print("Testing left motor backward...")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(3)
    stop_motors()
    time.sleep(1)

def test_right_motor_forward():
    print("Testing right motor forward...")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(3)
    stop_motors()
    time.sleep(1)

def test_right_motor_backward():
    print("Testing right motor backward...")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    time.sleep(3)
    stop_motors()
    time.sleep(1)

def test_both_motors_forward():
    print("Testing both motors forward...")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(3)
    stop_motors()
    time.sleep(1)

def test_both_motors_backward():
    print("Testing both motors backward...")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    time.sleep(60)
    stop_motors()
    time.sleep(1)

try:
    print("Starting motor test sequence")
    print("Each motor will run for 60 sec")
    ##test_left_motor_backward()
    ##test_right_motor_forward()
    ##test_right_motor_backward()
    ##test_both_motors_forward()
    test_both_motors_backward()

    print("Motor test sequence complete")

except KeyboardInterrupt:
    print("Test interrupted by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    stop_motors()
    GPIO.cleanup()
    print("GPIO pins cleaned up")
