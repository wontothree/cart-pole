import numpy as np
import casadi

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

from cartpole_nmpc import CartPoleNMPC

class Simulation():
    def __init__(self):
        self.cart_pole_nmpc = CartPoleNMPC()

    def simulate(self, current_state, time_span):
        time_steps = np.arange(time_span[0], time_span[1], self.cart_pole_nmpc.control_sampling_time)

        I = self.make_integrator()
        
        actual_state_trajectory = [current_state.full().ravel()]
        actual_control_trajectory = []

        for time_step in time_steps:
            optimal_state_trajectory, optimal_control_trajectory = self.cart_pole_nmpc.solve(current_state)

            current_state = I(x0=current_state, p=optimal_control_trajectory[0])["xf"]

            # 센서로부터 얻은 actual_state_trajectory를 저장해야 한다.
            actual_state_trajectory.append(current_state.full().ravel())
            actual_control_trajectory.append(optimal_control_trajectory[0])

        actual_state_trajectory.pop()

        return actual_state_trajectory, actual_control_trajectory, time_steps
    
    def visualize_table(self, actual_state_trajectory, actual_control_trajectory, time_steps):
        # Convert state and control trajectories to NumPy arrays
        actual_state_trajectory = np.array(actual_state_trajectory)
        actual_control_trajectory = np.array(actual_control_trajectory)
        
        # Ensure time_steps is a NumPy array
        time_steps = np.array(time_steps)
        
        # Plot states
        plt.figure(figsize=(12, 6))
        
        plt.subplot(1, 2, 1)
        for k in range(self.cart_pole_nmpc.state_dim):
            plt.plot(time_steps[:len(actual_state_trajectory)], actual_state_trajectory[:, k], label=f"x_{k}")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("State")
        plt.title("State Trajectories")
        
        # Plot controls
        plt.subplot(1, 2, 2)
        for k in range(self.cart_pole_nmpc.control_dim):
            plt.step(time_steps[:len(actual_control_trajectory)], actual_control_trajectory[:, k], linestyle="--", label=f"u_{k}")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Control")
        plt.title("Control Trajectories")

        plt.tight_layout()
        plt.show()

    def make_f(self):
        # constant parameter
        g = self.cart_pole_nmpc.gravity
        M = self.cart_pole_nmpc.cart_mass
        m = self.cart_pole_nmpc.pole_mass
        L = self.cart_pole_nmpc.pole_length

        # state variables
        x = self.cart_pole_nmpc.state[0]
        theta = self.cart_pole_nmpc.state[1]
        x_dot = self.cart_pole_nmpc.state[2]
        theta_dot = self.cart_pole_nmpc.state[3]

        # control input variable
        F = self.cart_pole_nmpc.control[0]

        sin = casadi.sin(theta)
        cos = casadi.cos(theta)
        common = M + m * sin**2

        # dynamics
        x_ddot = (m * g * sin * cos - m * L / 2 * sin * theta_dot**2 + F) / common
        theta_ddot = (- m * L / 2 * sin * cos * theta_dot**2 + (M + m) * g * sin + F * cos) / (L / 2 * common)

        states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

        continuous_dynamic_model = casadi.Function("f", [self.cart_pole_nmpc.state, self.cart_pole_nmpc.control], [states_dot], ['x', 'u'], ['x_dot'])

        return continuous_dynamic_model

    def make_integrator(self):
        # Integrator
        states = casadi.SX.sym("states", self.cart_pole_nmpc.state_dim)
        ctrls = casadi.SX.sym("ctrls", self.cart_pole_nmpc.control_dim)

        f = self.make_f()
        ode = f(x=states, u=ctrls)["x_dot"]

        dae = {"x": states, "p": ctrls, "ode": ode}

        I = casadi.integrator("I", "cvodes", dae, 0, self.cart_pole_nmpc.control_sampling_time)

        return I

    def animate(self, X, U, t_eval, filename="cart_pole.gif"):
        # Ensure X is a NumPy array for proper indexing
        X = np.array(X)
        
        # animation setup
        fig = plt.figure(figsize=(10, 10)) # size
        ax = fig.add_subplot(111)
        frames = np.arange(0, t_eval.size) # frame numbers
        fps = 1 / self.cart_pole_nmpc.control_sampling_time

        # update function for animation
        def update_figure(i):
            x_lim_min = -0.8
            x_lim_max = 0.8
            y_lim_min = -0.8
            y_lim_max = 0.8
            u_scale = 50

            # set up the axes
            ax.cla()
            ax.set_xlim(x_lim_min, x_lim_max)
            ax.set_ylim(y_lim_min, y_lim_max)
            ax.set_aspect("equal")
            ax.set_title(f"Cart Pole (Time = {t_eval[i]: 0.2f})", fontsize=20)
            ax.set_facecolor("#f0f0f0")  # Set background color

            # extract state and input values
            x, theta, x_dot, theta_dot = X[i]
            u, = U[i]

            # Normalize theta to the range [-pi, pi]
            theta = np.arctan2(np.sin(theta), np.cos(theta))

            # calculate pole coordinates
            points = np.array([
                [x, x - self.cart_pole_nmpc.pole_length * np.sin(theta)],
                [0, self.cart_pole_nmpc.pole_length * np.cos(theta)]
            ])

            # plot ground line
            ax.hlines(0, x_lim_min, x_lim_max, colors="black")

            # Plot the pole and its shadow
            ax.plot(*points, color="#c2a28c", lw=12, zorder=1)  # Pole
            ax.plot(*points, color="grey", lw=8, alpha=0.5, zorder=0)  # Pole shadow

            # arrow
            ax.arrow(x, 0, u / u_scale, 0, width=0.01, head_width=0.03, head_length=0.24, color="grey" if u > 0 else "grey")

            # cart
            w = 0.07  # width
            h = 0.07  # height
            rect = patches.Rectangle(xy=(x - w / 2, - h / 2), width=w, height=h, color="black")  # Cart color
            ax.add_patch(rect)

            # display state and input values
            state_and_input_text = (f"position of cart: {x:.2f} m\n"
                        f"angle of pole: {theta:.2f} rad\n"
                        f"velocity of cart: {x_dot:.2f} m/s\n"
                        f"angular velocity of pole: {theta_dot:.2f} rad/s\n"
                        f"force: {u:.2f} N")
            ax.text(x_lim_max - 0.85, y_lim_max - 0.3, state_and_input_text, fontsize=15,
                    bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))

        # Create and save the animation
        ani = FuncAnimation(fig, update_figure, frames=frames)
        ani.save(filename, writer="pillow", fps=fps)

if __name__ == "__main__":
    simulation = Simulation()

    simulation.cart_pole_nmpc.set_target_state(casadi.DM([0, 0, 0, 0]))
    simulation.cart_pole_nmpc.set_target_control(casadi.DM([0]))
    current_state = casadi.DM([0, np.pi, 0, 0])

    actual_state_trajectory, actual_control_trajectory, time_steps = simulation.simulate(current_state, [0, 5])
    # simulation.visualize_table(actual_state_trajectory, actual_control_trajectory, time_steps)
    simulation.animate(actual_state_trajectory, actual_control_trajectory, time_steps, filename="./imgs/cart_pole.gif")