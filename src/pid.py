class PID:
    def __init__(self, kp, ki, kd, output_min=-5, output_max=5):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0
        self.prev_error = 0
        self.first_run = True

    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.first_run = True

    def update(self, error, dt):
        # P term
        P = self.kp * error

        # I term (with anti-windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, 50), -50)  # clamp integral
        I = self.ki * self.integral

        # D term
        if self.first_run:
            D = 0
            self.first_run = False
        else:
            D = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        # Output
        output = P + I + D

        # Clamp output
        output = max(self.output_min, min(output, self.output_max))

        return output
