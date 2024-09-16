import serial

class UartReceiver:
    def __init__(self, port1='COM5', baudrate1=38400, port2='COM8', baudrate2=9600, timeout=0.01):
        # set serial port
        # self.serial1 = serial.Serial(port1, baudrate1, timeout=timeout)
        self.serial2 = serial.Serial(port2, baudrate2, timeout=timeout)

        self.cart_previous_

    def receive_pole_angle_and_angular_velocity(self):
        # check if data is available in the buffer
        if self.serial1.in_waiting > 0:
            data = self.serial1.readline().decode('utf-8').strip()
            data = data.split(',')

            pole_angle = float(data[0])
            pole_angular_velocity = float(data[1])

            return pole_angle, pole_angular_velocity
    
    def receive_stepper_motor_tick_time(self):
        if self.serial2.in_waiting > 0:
            data = self.serial2.readline().decode('utf-8').strip()
            data = data.split(',')
            stepper_motor_tick = float(data[0])
            stepper_motor_tick_observation_time = float(data[1])


        
    

if __name__ == "__main__":
    uart_receiver = UartReceiver()

    try:
        while True:
            # receive_uart_return = uart_receiver.receive_pole_angle_and_angular_velocity()
            # if receive_uart_return is not None:
            #     pole_angle, pole_angular_velocity = receive_uart_return
            #     print(pole_angle, pole_angular_velocity)
            
            receive_stepper_motor_tick_return = uart_receiver.receive_stepper_motor_tick()
            if receive_stepper_motor_tick_return is not None:
                print(receive_stepper_motor_tick_return)

    except serial.SerialException as e:
        print(f"SerialException: {e}")
    finally:
        if uart_receiver.serial1.is_open:
            uart_receiver.serial1.close()
        if uart_receiver.serial2.is_open:
            uart_receiver.serial2.close()
