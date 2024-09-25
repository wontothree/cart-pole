import serial
import casadi

class Uart:
    def __init__(self):
        # set serial port
        self.serial1 = serial.Serial(port='COM5', baudrate=38400, timeout=1)
        self.serial2 = serial.Serial(port='COM7', baudrate=1000000, timeout=1)

        self.STEPS_PER_METERS = 6366 # 6366.198

        # state
        self.cart_position = 0
        self.pole_angle = 0
        self.cart_velocity = 0
        self.pole_angular_velocity = 0

        self.stepper_motor_previous_tick = 0
        self.stepper_motor_previous_tick_observation_time = 0

    # receive pole angle and angular velocity from Arduino UNO R4 WiFi
    def receive_pole_angle_and_angular_velocity(self):
        # check if data is available in the buffer
        if self.serial1.in_waiting > 0:
            data = self.serial1.readline().decode('utf-8').strip()
            data = data.split(',')

            pole_angle = float(data[0])
            pole_angular_velocity = float(data[1])

            return pole_angle, pole_angular_velocity
    
    # receive cart position and velocity from Arduino UNO R3
    def receive_cart_position_and_velocity(self):
        if self.serial2.in_waiting > 0:
            data = self.serial2.readline().decode('utf-8').strip()

            v_index = data.index('V') + 1
            p_index = data.index('P')
            velocity = float(data[v_index:p_index])
            position = float(data[p_index + 1:])
            
            return position, velocity

    def send_current_optimal_action(self, current_optimal_action):
        INVERSE_SUM_MASS = 4.7
        
        current_optimal_acceleration = str(int(INVERSE_SUM_MASS * current_optimal_action * 100))

        current_optimal_acceleration += '\n'

        # for _ in range(10):
        self.serial2.write(current_optimal_acceleration.encode())

    def send_acceleration(self):
        acceleration = input("가속도를 입력하세요: ")
        acceleration += '\n'
        print(acceleration.encode())

        # for _ in range(10):
        uart.serial2.write(acceleration.encode())

if __name__ == "__main__":
    uart = Uart()

    if not uart.serial2.is_open:
        uart.serial2.open()
        
    print(f"Serial2 is open: {uart.serial2.is_open}")
    print(uart.serial2.portstr)

    while (1):
        try:
            uart.send_acceleration()

        except Exception as e:
            print(f"Error: {e}")
