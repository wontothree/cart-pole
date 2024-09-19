import serial

class Uart:
    def __init__(self):
        # set serial port
        self.serial1 = serial.Serial(port='COM5', baudrate=38400, timeout=1)
        self.serial2 = serial.Serial(port='COM8', baudrate=9600, timeout=1)

        self.STEPS_PER_METERS = 6366 # 6366.198

        # state
        self.cart_position = 0
        self.pole_angle = 0
        self.cart_velocity = 0
        self.pole_angular_velocity = 0

        self.stepper_motor_previous_tick = 0
        self.stepper_motor_previous_tick_observation_time = 0

    def receive_pole_angle_and_angular_velocity(self):
        # check if data is available in the buffer
        if self.serial1.in_waiting > 0:
            data = self.serial1.readline().decode('utf-8').strip()
            data = data.split(',')

            pole_angle = float(data[0])
            pole_angular_velocity = float(data[1])

            return pole_angle, pole_angular_velocity
    
    def receive_stepper_motor_tick(self):
        if self.serial2.in_waiting > 0:
            data = self.serial2.readline().decode('utf-8').strip()
            data = data.split()
            stepper_motor_tick = float(data[0])
            stepper_motor_tick_observation_time = float(data[1])

            return stepper_motor_tick, stepper_motor_tick_observation_time

    def calculate_cart_position_and_velocity(self, stepper_motor_tick, stepper_motor_tick_observation_time):
        if (stepper_motor_tick_observation_time - self.stepper_motor_previous_tick_observation_time):
            cart_position = stepper_motor_tick / self.STEPS_PER_METERS # m
            cart_velocity = ((stepper_motor_tick - self.stepper_motor_previous_tick) / self.STEPS_PER_METERS / ((stepper_motor_tick_observation_time - self.stepper_motor_previous_tick_observation_time) / 1000.0)) # m/s

            self.stepper_motor_previous_tick = stepper_motor_tick
            self.stepper_motor_previous_tick_observation_time = stepper_motor_tick_observation_time

            return cart_position, cart_velocity

if __name__ == "__main__":
    uart = Uart()
    try:
        while True:
            receive_pole_angle_and_angular_velocity_return = uart.receive_pole_angle_and_angular_velocity()            
            if receive_pole_angle_and_angular_velocity_return is not None:
                uart.pole_angle, uart.pole_angular_velocity = receive_pole_angle_and_angular_velocity_return

                # print(pole_angle, pole_angular_velocity)

            receive_stepper_motor_tick_return = uart.receive_stepper_motor_tick()
            if receive_stepper_motor_tick_return is not None:
                stepper_motor_tick, stepper_motor_tick_observation_time = receive_stepper_motor_tick_return

                uart.cart_position, uart.cart_velocity = uart.calculate_cart_position_and_velocity(stepper_motor_tick, stepper_motor_tick_observation_time)

                # print(cart_position, cart_velocity)

            print(uart.cart_position, uart.pole_angle, uart.cart_velocity, uart.pole_angular_velocity)


    except serial.SerialException as e:
        print(f"SerialException: {e}")
    finally:
        if uart.serial1.is_open:
            uart.serial1.close()
        if uart.serial2.is_open:
            uart.serial2.close()
