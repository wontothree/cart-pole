import serial
import time

class CartPoleStateEstimator:
    def __init__(self, port='COM5', baudrate=38400, timeout=0.01):
        # set serial port
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

        self.cart_position = None
        self.cart_velocity = None
        self.pole_current_angle = None
        self.pole_previous_angle = None
        self.pole_angular_velocity = None

        self.observe_previous_time = time.time()

    def measure_pole_angle(self):
        try:
            while True:
                # is data in buffer
                if self.serial.in_waiting > 0:
                    # read data
                    encoder_data = self.serial.readline().decode('utf-8').strip()
                    print(encoder_data)

                    # if encoder_data:
                    #     # encoder_data = encoder_data.split(',')

                    #     if len(encoder_data) == 3:
                    #         # self.pole_current_angle = float(encoder_data[0].strip().replace("Degree: ", ''))
                    #         # print(self.pole_current_angle)



                            # # test
                            # if self.pole_previous_angle is None:
                            #     self.pole_previous_angle = self.pole_current_angle
                            # elif self.pole_previous_angle is not None:
                            #     observe_current_time = time.time()
                            #     delta_time = observe_current_time - self.observe_previous_time

                            #     if delta_time > 0:
                            #         self.pole_angular_velocity = (self.pole_current_angle - self.pole_previous_angle) / delta_time

                            #         print(f"Angular Velocity of Pole: {self.pole_angular_velocity} degrees per second")
                            
                            #     self.observe_previous_time = observe_current_time
                            #     self.pole_previous_angle = self.pole_current_angle

        except serial.SerialException as e:
            print(f"SerialException: {e}")
        finally:
            if self.serial.is_open:
                self.serial.close()

    def estimate_pole_angular_velocity(self):
        if self.pole_previous_angle is not None:
            observe_current_time = time.time()
            delta_time = observe_current_time - self.observe_previous_time

            if delta_time > 0:
                self.angular_velocity = (self.pole_current_angle - self.pole_previous_angle) / delta_time

                print(f"Angular Velocity of Pole: {self.pole_angular_velocity:.2f} degrees per second")
        
            self.observe_previous_time = observe_current_time
            self.pole_previous_angle = self.pole_current_angle

    def estimate_cart_position(self):
        pass

    def estimate_cart_velocity(self):
        pass

if __name__ == "__main__":
    estimator = CartPoleStateEstimator()
    estimator.measure_pole_angle()
    # estimator.estimate_pole_angular_velocity()
