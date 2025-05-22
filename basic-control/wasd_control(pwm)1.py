#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys
import tty
import termios
import time
import select
import threading

# ---------------- GPIO pin assignments ----------------
IN1 = 17          # Left motor forward
IN2 = 18          # Left motor backward
IN3 = 22          # Right motor forward
IN4 = 23          # Right motor backward
PWM_FREQ = 100    # Hz – enough for DC motors without audible whine

# -------------- GPIO / PWM initialisation -------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in (IN1, IN2, IN3, IN4):
    GPIO.setup(pin, GPIO.OUT)

pwm_IN1 = GPIO.PWM(IN1, PWM_FREQ)
pwm_IN2 = GPIO.PWM(IN2, PWM_FREQ)
pwm_IN3 = GPIO.PWM(IN3, PWM_FREQ)
pwm_IN4 = GPIO.PWM(IN4, PWM_FREQ)

for pwm in (pwm_IN1, pwm_IN2, pwm_IN3, pwm_IN4):
    pwm.start(0)                     # motors initially stopped

# ------------------- globals ---------------------------
speed = 50        # duty-cycle %, start at 50
current_motion = "stopped"
last_command_time = 0
timeout_duration = 0.3  # Stop robot after 300ms of no input
running = True

# ---------------- helper functions --------------------
def get_char_non_blocking():
    """Get character without blocking, returns None if no input available"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1).lower()
    return None

def _apply(left_fwd, left_back, right_fwd, right_back):
    """Set duty-cycle for each channel in a single place."""
    pwm_IN1.ChangeDutyCycle(left_fwd)
    pwm_IN2.ChangeDutyCycle(left_back)
    pwm_IN3.ChangeDutyCycle(right_fwd)
    pwm_IN4.ChangeDutyCycle(right_back)

def stop_motors():
    global current_motion
    _apply(0, 0, 0, 0)
    current_motion = "stopped"
    print("Motors stopped")

def forward():
    global current_motion, last_command_time
    _apply(speed, 0, speed, 0)
    current_motion = "forward"
    last_command_time = time.time()
    print(f"Forward at {speed}%")

def backward():
    global current_motion, last_command_time
    _apply(0, speed, 0, speed)
    current_motion = "backward"
    last_command_time = time.time()
    print(f"Backward at {speed}%")

def turn_left():
    global current_motion, last_command_time
    _apply(0, 0, speed, 0)
    current_motion = "left"
    last_command_time = time.time()
    print(f"Left at {speed}%")

def turn_right():
    global current_motion, last_command_time
    _apply(speed, 0, 0, 0)
    current_motion = "right"
    last_command_time = time.time()
    print(f"Right at {speed}%")

def increase_speed(step=10):
    global speed
    speed = min(100, speed + step)
    print(f"Speed ↑ → {speed}%")
    refresh_motion()

def decrease_speed(step=10):
    global speed
    speed = max(0, speed - step)
    print(f"Speed ↓ → {speed}%")
    refresh_motion()

def refresh_motion():
    """Re-issue the last motion command so speed change takes effect immediately."""
    motions = {
        "forward": forward,
        "backward": backward,
        "left": turn_left,
        "right": turn_right,
        "stopped": stop_motors
    }
    if current_motion != "stopped":
        motions[current_motion]()

def timeout_monitor():
    """Monitor for timeout and stop motors if no recent commands"""
    global running, current_motion
    while running:
        current_time = time.time()
        if (current_motion != "stopped" and 
            current_time - last_command_time > timeout_duration):
            stop_motors()
        time.sleep(0.05)  # Check every 50ms

# ------------------- main loop ------------------------
try:
    # Set up non-blocking input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    
    # Start timeout monitoring thread
    timeout_thread = threading.Thread(target=timeout_monitor, daemon=True)
    timeout_thread.start()
    
    print(
        "\nWASD Motor Control with PWM speed (Auto-Stop Enabled)\n"
        "----------------------------------------------------\n"
        "W: Forward      A: Left\n"
        "S: Backward     D: Right\n"
        "+: Faster       -: Slower\n"
        "Space: Stop     Q: Quit\n"
        f"Auto-stop timeout: {timeout_duration}s\n"
        "Hold down keys for continuous movement!\n"
    )
    
    while running:
        char = get_char_non_blocking()
        
        if char:
            if char == 'w':
                forward()
            elif char == 's':
                backward()
            elif char == 'a':
                turn_left()
            elif char == 'd':
                turn_right()
            elif char == '+':
                increase_speed()
            elif char == '-':
                decrease_speed()
            elif char == ' ':
                stop_motors()
            elif char == 'q':
                running = False
                break
        
        time.sleep(0.02)  # Small delay to prevent excessive CPU usage

except KeyboardInterrupt:
    print("\nProgram interrupted by user")
except Exception as e:
    print(f"\nUnhandled error: {e}")
finally:
    running = False
    stop_motors()
    
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    for pwm in (pwm_IN1, pwm_IN2, pwm_IN3, pwm_IN4):
        pwm.stop()
    GPIO.cleanup()
    print("GPIO cleanup complete – exiting.")