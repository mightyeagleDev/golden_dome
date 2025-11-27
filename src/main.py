from gpiozero import Servo
from time import sleep

SERVO_PIN = 18  # GPIO18 supports hardware PWM on Pi 5

# Use wide pulse range for Pi 5 + hobby servos
servo = Servo(
    18,
    min_pulse_width=0.40/1000,   # slightly lower
    max_pulse_width=2.55/1000    # slightly higher
)
print("Servo ready")

try:
    while True:
        print("Moving to 0°")
        servo.value = -1
        sleep(1)


        print("Moving to 180°")
        servo.value = 1
        sleep(1)

except KeyboardInterrupt:
    print("Stopped")
finally:
    servo.close()
