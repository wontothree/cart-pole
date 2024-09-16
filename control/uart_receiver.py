import serial
import time

class UartReceiver:
    def __init__(self, port='COM5', baudrate=38400, timeout=0.01):
        # set serial port
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def receive_uart(self):
        # check if data is available in the buffer
        if self.serial.in_waiting > 0:
            data = self.serial.readline().decode('utf-8').strip()
            data = data.split(',')

            pole_angle = float(data[0])
            pole_angular_velocity = float(data[1])

            return pole_angle, pole_angular_velocity

if __name__ == "__main__":
    uart_receiver = UartReceiver()

    try:
        while True:
            receive_uaet_return = uart_receiver.receive_uart()
            if receive_uaet_return is not None:
                pole_angle, pole_angular_velocity = receive_uaet_return
                print(pole_angle, pole_angular_velocity)

    except serial.SerialException as e:
        print(f"SerialException: {e}")
    finally:
        if uart_receiver.serial.is_open:
            uart_receiver.serial.close()
