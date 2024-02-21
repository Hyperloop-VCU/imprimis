# Input: desired pulses per loop (int) and direction (bool), actual pulses per loop (int) and direction (bool)
# Output: PWM duty cycle (int, 0-100) to control speed, single bit to control direction (bool)

class PIDController:
    def __init__(self, kP=0, kI=0, kD=0, dt=0.05):
        """PID controller object.
        :param kP: proportional gain
        :param kI: integral gain
        :param kD: derivative gain
        :param dt: time between PID updates (1 / loops per second)"""

        self.dt = dt
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_error = 0
        self.integral = 0
        self.speed_output = 0
        self.direction_output = True

    def update_output(self, setSpeed, setDir, actualSpeed, actualDir):
        error = (1 if setDir else -1)*setSpeed - (1 if actualDir else -1)*actualSpeed

        if self.prev_error * error < 0:  # Zero the integral when the error crosses zero (changes sign)
            self.integral = 0

        P = self.kP * error
        self.integral += error * self.dt
        D = (error - self.prev_error) / self.dt

        PID = P + self.integral * self.kI + D
        self.speed_output = min(100, max(0, PID))
        self.direction_output = actualDir if setDir == actualDir else not actualDir
        self.prev_error = error



