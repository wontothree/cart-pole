import time

class CartPolePID():
    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0

        # radian (not degree)
        self.target_angle = 0

        self.last_time = 0
        self.last_angle_error = 0
    
    def set_target_angle(self, target_angle):
        # radian (-pi ~ pi)
        self.target_angle = target_angle

    def set_pid(self, KP, KI, KD):
        self.kp = KP
        self.ki = KI
        self.kd = KD

    # calculate next action from current state and target state
    def control(self, angle):
        # proportional term
        angle_error = abs(angle - self.target_angle)

        current_time = time.time()
        time_interval = current_time - self.last_time

        # integral term
        cumulative_angle_error = angle_error * time_interval

        # derivative term
        derivative_angle_error = (angle_error - self.last_angle_error) / time_interval

        # output of pid
        output = self.kp * angle_error + self.ki * cumulative_angle_error + self.kd * derivative_angle_error

        # update last values
        self.last_angle_error = angle_error
        self.last_time = current_time

        return output
        
if __name__ == "__main__":
    cartpole_pid = CartPolePID()

    cartpole_pid.set_target_angle(0)

    cartpole_pid.set_pid(1, 0, 0)

    print(cartpole_pid.control(2))
    time.sleep(1)
    print(cartpole_pid.control(1))
