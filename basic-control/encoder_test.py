#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import threading

# ============== PIN CONFIGURATION ==============
# Motor driver pins
MOTOR_IN1 = 18  # Motor forward
MOTOR_IN2 = 17  # Motor backward

# Encoder pins
ENCODER_A = 24
ENCODER_B = 25

# ============== GLOBAL VARIABLES ==============
# Motor state tracking
motor_running = False
motor_direction = "stop"  # "forward", "backward", or "stop"

# Encoder counting
encoder_count = 0
pulses_per_second = 0

# ============== GPIO SETUP ==============
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup motor pins as outputs
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)

# Stop motors initially
GPIO.output(MOTOR_IN1, GPIO.LOW)
GPIO.output(MOTOR_IN2, GPIO.LOW)

# Setup encoder pins with both pull-up and detection
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ============== UTILITY FUNCTIONS ==============
def getch():
    """Get a single character from stdin without waiting for enter key"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ============== MOTOR CONTROL FUNCTIONS ==============
def motor_stop():
    """Stop the motor"""
    global motor_running, motor_direction
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    motor_running = False
    motor_direction = "stop"
    print("\nMotor: STOPPED")

def motor_forward():
    """Run motor forward"""
    global motor_running, motor_direction
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    motor_running = True
    motor_direction = "forward"
    print("\nMotor: FORWARD")

def motor_backward():
    """Run motor backward"""
    global motor_running, motor_direction
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.HIGH)
    motor_running = True
    motor_direction = "backward"
    print("\nMotor: BACKWARD")

# ============== ENCODER FUNCTIONS ==============
def encoder_callback(channel):
    """Callback function triggered on encoder pulse"""
    global encoder_count
    
    # Increment the counter on each rising edge of Encoder A
    if channel == ENCODER_A and GPIO.input(ENCODER_A) == 1:
        encoder_count += 1
        # Uncomment the next line for debugging
        # print(f"Encoder pulse detected! Count: {encoder_count}")

# Register encoder callback (detect rising edge only)
GPIO.add_event_detect(ENCODER_A, GPIO.RISING, callback=encoder_callback)

# ============== RPM CALCULATION THREAD ==============
def rpm_calculator():
    """Thread function to calculate RPM based on encoder pulses"""
    global encoder_count, pulses_per_second
    
    # N20 Motor configuration (adjust these for your specific motor)
    PULSES_PER_REVOLUTION = 12    # Typically 12 for most simple encoders
    GEAR_RATIO = 100              # Adjust to your N20 motor's gear ratio (50, 100, 298, etc.)
    
    while True:
        # Record starting values
        start_count = encoder_count
        start_time = time.time()
        
        # Wait for a short period
        time.sleep(0.5)  # Calculate twice per second for more responsive readings
        
        # Calculate pulses during this period
        elapsed_time = time.time() - start_time
        pulses = encoder_count - start_count
        
        # Calculate pulses per second and RPM
        if elapsed_time > 0:
            pulses_per_second = pulses / elapsed_time
            rpm = (pulses_per_second * 60) / (PULSES_PER_REVOLUTION * GEAR_RATIO)
            
            # Print status with details (only if motor is running)
            if motor_running:
                print(f"\rMotor: {motor_direction.upper()} | "
                      f"Encoder: {encoder_count} pulses | "
                      f"Speed: {pulses_per_second:.1f} pulses/sec | "
                      f"RPM: {rp:.2f}", end="")

# ============== USER INTERFACE FUNCTION ==============
def show_instructions():
    """Display user instructions"""
    print("\n======== N20 MOTOR ENCODER TEST ========")
    print("Controls:")
    print("  W - Run motor forward")
    print("  S - Run motor backward")
    print("  Space - Stop motor")
    print("  R - Reset encoder counter")
    print("  Q - Quit program")
    print("========================================")
    print("Encoder data will be displayed when the motor is running.")
    print("Waiting for commands...")

# ============== MAIN PROGRAM ==============
if __name__ == "__main__":
    try:
        # Start the RPM calculator thread
        rpm_thread = threading.Thread(target=rpm_calculator, daemon=True)
        rpm_thread.start()
        
        # Show instructions
        show_instructions()
        
        # Main control loop
        while True:
            # Get user input
            key = getch().lower()
            
            # Process commands
            if key == 'w':
                motor_forward()
            elif key == 's':
                motor_backward()
            elif key == ' ':
                motor_stop()
            elif key == 'r':
                encoder_count = 0
                print("\nEncoder counter reset to zero.")
            elif key == 'q':
                print("\nExiting program...")
                break
            
            # Short delay to prevent CPU hogging
            time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        # Clean up and exit
        motor_stop()
        GPIO.cleanup()
        print("GPIO pins cleaned up. Program ended.")
