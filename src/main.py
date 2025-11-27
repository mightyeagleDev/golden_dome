from gpiozero import Servo
from time import sleep

SERVO_PIN = 18  # GPIO18 supports hardware PWM on Pi 5

# Use wide pulse range for Pi 5 + hobby servos
servo = Servo(
    SERVO_PIN,
    min_pulse_width=0.5/1000,   # 0.5 ms => 0°
    max_pulse_width=2.5/1000    # 2.5 ms => 180°
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
