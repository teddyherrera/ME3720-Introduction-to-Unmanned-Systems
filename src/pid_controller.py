

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def depth_update(self, current_value, desired_value, dt):
        error = desired_value - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output, error

    def heading_update(self, current_angle, desired_angle, dt):
        # Function to handle angle wrapping and PID control for angles
        unwrapped_error = desired_angle - current_angle
        error = self.wrap_to_180(unwrapped_error)
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output, error

    @staticmethod
    def wrap_to_180(angle):
        # Function to wrap angle to [-180, 180]
        return (angle + 180) % 360 - 180
