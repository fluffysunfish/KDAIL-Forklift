import RPi.GPIO as GPIO
import time
from collections import deque

# Define encoder pins (Adjust these based on your wiring)
ENCODER_A = 24  # Channel A (C1)
ENCODER_B =25  # Channel B (C2)

# Encoder variables
encoder_count = 0
last_encoded = 0
debounce_window = deque(maxlen=5)  # Stores last 5 readings for filtering

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read_encoder(channel):
    global encoder_count, last_encoded

    # Read current state of both channels
    A = GPIO.input(ENCODER_A)
    B = GPIO.input(ENCODER_B)

    # Store in debounce window
    debounce_window.append((A, B))
    
    # Check if we have stable readings (debounce filter)
    if len(set(debounce_window)) > 1:
        return  # Ignore noisy transitions
    
    # Quadrature decoding logic
    encoded = (A << 1) | B
    delta = (encoded - last_encoded) % 4
    
    if delta == 1:
        encoder_count += 1  # Forward
    elif delta == 3:
        encoder_count -= 1  # Backward
    
    last_encoded = encoded
    print(f"Encoder Count: {encoder_count}")

# Attach interrupts (detecting both rising and falling edges)
GPIO.add_event_detect(ENCODER_A, GPIO.BOTH, callback=read_encoder)
GPIO.add_event_detect(ENCODER_B, GPIO.BOTH, callback=read_encoder)

# Run indefinitely
try:
    print("Reading encoder data... Press Ctrl+C to stop.")
    while True:
        time.sleep(0.1)  # Reduce CPU usage

except KeyboardInterrupt:
    print("\nExiting...")
    GPIO.cleanup()

