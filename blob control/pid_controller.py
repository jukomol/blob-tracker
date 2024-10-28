import config

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        """
        Initialize PID controller with gains and initial values.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.reset()

    def reset(self):
        """
        Reset PID control variables.
        """
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

    def compute(self, error_x, error_y):
        """
        Compute the PID output for both x and y axes.
        Returns a tuple: (output_x, output_y)
        """
        # PID calculation for X-axis
        proportional_x = self.Kp * error_x
        self.integral_x += error_x * self.dt
        derivative_x = (error_x - self.prev_error_x) / self.dt
        output_x = proportional_x + (self.Ki * self.integral_x) + (self.Kd * derivative_x)
        self.prev_error_x = error_x

        # PID calculation for Y-axis
        proportional_y = self.Kp * error_y
        self.integral_y += error_y * self.dt
        derivative_y = (error_y - self.prev_error_y) / self.dt
        output_y = proportional_y + (self.Ki * self.integral_y) + (self.Kd * derivative_y)
        self.prev_error_y = error_y

        return output_x, output_y
