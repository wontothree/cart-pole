import serial
import time

class CartPoleStateObserver:
    def __init__(self, port='COM5', baudrate=38400, timeout=0.01):
        # set serial port
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

        self.pole_previous_angle = None
        self.pole_previous_angle_observing_time = None

    def measure_pole_angle(self):
        # check if data is available in the buffer
        if self.serial.in_waiting > 0:
            # read data from the serial port
            pole_angle_from_encoder = float(self.serial.readline().decode('utf-8').strip())
            pole_angle = pole_angle_from_encoder
            pole_angle_observing_time = time.time()

            return pole_angle, pole_angle_observing_time

    def estimate_pole_angular_velocity(self, pole_angle, pole_angle_observing_time):
        if self.pole_previous_angle_observing_time is None:
            self.pole_previous_angle = pole_angle
            self.pole_previous_angle_observing_time = pole_angle_observing_time
        else:
            delta_time = pole_angle_observing_time - self.pole_previous_angle_observing_time
            if delta_time > 0:
                pole_angular_velocity = (pole_angle - self.pole_previous_angle) / delta_time

                self.pole_previous_angle = pole_angle
                self.pole_previous_angle_observing_time = pole_angle_observing_time

                return pole_angular_velocity

    def estimate_cart_position(self):
        pass

    def estimate_cart_velocity(self):
        pass

if __name__ == "__main__":
    state_observer = CartPoleStateObserver()

    try:
        while True:
            measure_pole_angle_return = state_observer.measure_pole_angle()
            if measure_pole_angle_return is not None:
                pole_angle, pole_angle_observing_time = measure_pole_angle_return
                pole_angular_velocity = state_observer.estimate_pole_angular_velocity(pole_angle, pole_angle_observing_time)
                print(pole_angular_velocity)

    except serial.SerialException as e:
        print(f"SerialException: {e}")
    finally:
        if state_observer.serial.is_open:
            state_observer.serial.close()
