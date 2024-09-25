import serial
import casadi
import numpy as np

from uart import Uart
from cartpole_nmpc import CartPoleNMPC

if __name__ == "__main__":
    uart = Uart()
    cartpole_nmpc = CartPoleNMPC()
    try:
        while True:
            # print pole angle and angular velocity
            receive_pole_angle_and_angular_velocity_return = uart.receive_pole_angle_and_angular_velocity()            
            if receive_pole_angle_and_angular_velocity_return is not None:
                uart.pole_angle, uart.pole_angular_velocity = receive_pole_angle_and_angular_velocity_return
                # print(uart.pole_angle, uart.pole_angular_velocity)

            receive_cart_position_and_velocity_return = uart.receive_cart_position_and_velocity()
            if receive_cart_position_and_velocity_return is not None:
                uart.cart_position, uart.cart_velocity = receive_cart_position_and_velocity_return
                print(uart.position, uart.velocity)

            current_state = casadi.DM([uart.cart_position, uart.pole_angle, uart.cart_velocity, uart.pole_angular_velocity])

            current_state = casadi.DM([uart.cart_position, uart.pole_angle, uart.cart_velocity, uart.pole_angular_velocity])
            print(current_state)

            # set target state and target action
            cartpole_nmpc.set_target_state(casadi.DM([0, 0, 0, 0]))
            cartpole_nmpc.set_target_control(casadi.DM([0]))

            optimal_state_trajectory, optimal_control_trajectory = cartpole_nmpc.solve(current_state)
            current_optimal_action = optimal_control_trajectory[0]

            # print(current_optimal_action)

            # uart.send_current_optimal_action(current_optimal_action)
            
    except serial.SerialException as e:
        print(f"SerialException: {e}")
    finally:
        if uart.serial1.is_open:
            uart.serial1.close()
        if uart.serial2.is_open:
            uart.serial2.close()
