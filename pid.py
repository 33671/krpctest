class PidController:
    def __init__(self, kp, ki, kd, setpoint,max_integral=10000000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0
        self.max_integral = max_integral  # 最大积分值

    def set_maximum_integral(self, max_integral):
        """设置最大积分值"""
        self.max_integral = max_integral

    def compute(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        if self.integral > self.max_integral or self.integral < -self.max_integral:
            print(f"Integral value {self.integral} exceeds limits, clamping to {self.max_integral}")
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)  # 限制积分值
        derivative = (error - self.previous_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.previous_error = error

        return output
    
    def clamp_compute(self, measurement, dt, min_output, max_output):
        """计算PID输出并限制在指定范围内"""
        output = self.compute(measurement, dt)
        return max(min(output, max_output), min_output)
    
    def reset(self):
        """重置PID控制器的状态"""
        self.previous_error = 0
        self.integral = 0
    def set_setpoint(self, setpoint):
        """设置新的设定点"""
        self.setpoint = setpoint
        self.reset()