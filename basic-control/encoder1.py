import RPi.GPIO as GPIO
import time

# Define encoder pin (only one working channel)
ENCODER_A = 25  # Channel A (C1)

# Initialize pulse count
pulse_count = 0

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def encoder_callback(channel):
    """Interrupt handler for encoder pulses."""
    global pulse_count
    pulse_count += 1  # Increment on every rising or falling edge
    print(f"Pulse Count: {pulse_count}")

# Attach interrupt to Channel A (detect both rising & falling edges)
GPIO.add_event_detect(ENCODER_A, GPIO.BOTH, callback=encoder_callback)

# Run indefinitely
try:
    print("Counting pulses... Press Ctrl+C to stop.")
    while True:
        time.sleep(1)  # Reduce CPU usage

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    GPIO.cleanup()

