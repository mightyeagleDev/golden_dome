from gpiozero import Servo
from time import sleep

# Define the GPIO pin connected to the servo's signal line
# We'll use BCM pin 18 (physical pin 12)
SERVO_PIN = 18

# Initialize the Servo object
# The default min_pulse_width and max_pulse_width usually work well 
# for standard hobby servos (like SG90 or MG995)
servo = Servo(SERVO_PIN)

print(f"Servo initialized on GPIO {SERVO_PIN}. Press Ctrl+C to exit.")

try:
    while True:
        # Move to the maximum position (180 degrees, or 1.0 duty cycle value)
        print("Moving to max...")
        servo.max()
        sleep(1) 
        
        # Move to the minimum position (0 degrees, or -1.0 duty cycle value)
        print("Moving to min...")
        servo.min()
        sleep(1)
        
        # Move to the mid position (90 degrees, or 0.0 duty cycle value)
        print("Moving to mid...")
        servo.mid()
        sleep(1)
        
        # You can also set a specific value between -1 and 1
        print("Moving to 45 degrees (approx)...")
        servo.value = 0.5
        sleep(1)

except KeyboardInterrupt:
    print("\nProgram stopped. Cleaning up.")
finally:
    # Cleanup is implicitly handled when the script exits, 
    # but good practice to close if needed.
    # We can explicitly set it to a known state if desired:
    servo.close()