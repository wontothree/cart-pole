# import casadi
# import numpy as np

# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# from matplotlib.animation import FuncAnimation

# from cartpole_nmpc import CartPoleNMPC

# class Visualize(CartPoleNMPC):
#     def simulate(self, current_state, time_span):
#         time_steps = np.arange(time_span[0], time_span[1], self.control_sampling_time)

#         # ...
#         I = self.make_integrator()
        
#         actual_state_trajectory = [current_state.full().ravel()]
#         actual_control_trajectory = []

#         for time_step in time_steps:
#             optimal_state_trajectory, optimal_control_trajectory = self.solve(current_state)

#             current_state = I(x0=current_state, p=optimal_control_trajectory[0])["xf"]

#             # 센서로부터 얻은 actual_state_trajectory를 저장해야 한다.
#             # actual_state_trajectory.append(optimal_state_trajectory[0])
#             actual_state_trajectory.append(current_state.full().ravel())
#             actual_control_trajectory.append(optimal_control_trajectory[0])

#         actual_state_trajectory.pop()

#         return actual_state_trajectory, actual_control_trajectory, time_steps
    
#     def visualize(self, actual_state_trajectory, actual_control_trajectory, time_steps):
#         # Convert state and control trajectories to NumPy arrays
#         actual_state_trajectory = np.array(actual_state_trajectory)
#         actual_control_trajectory = np.array(actual_control_trajectory)
        
#         # Ensure time_steps is a NumPy array
#         time_steps = np.array(time_steps)
        
#         # Plot states
#         plt.figure(figsize=(12, 6))
        
#         plt.subplot(1, 2, 1)
#         for k in range(self.state_dim):
#             plt.plot(time_steps[:len(actual_state_trajectory)], actual_state_trajectory[:, k], label=f"x_{k}")
#         plt.legend()
#         plt.xlabel("Time")
#         plt.ylabel("State")
#         plt.title("State Trajectories")
        
#         # Plot controls
#         plt.subplot(1, 2, 2)
#         for k in range(self.control_dim):
#             plt.step(time_steps[:len(actual_control_trajectory)], actual_control_trajectory[:, k], linestyle="--", label=f"u_{k}")
#         plt.legend()
#         plt.xlabel("Time")
#         plt.ylabel("Control")
#         plt.title("Control Trajectories")

#         plt.tight_layout()
#         plt.show()

#     def make_f(self):
#         # constant parameter
#         g = self.gravity
#         M = self.cart_mass
#         m = self.pole_mass
#         L = self.pole_length

#         # state variables
#         x = self.state[0]
#         theta = self.state[1]
#         x_dot = self.state[2]
#         theta_dot = self.state[3]

#         # control input variable
#         F = self.control[0]

#         sin = casadi.sin(theta)
#         cos = casadi.cos(theta)
#         common = M + m * sin**2

#         # dynamics
#         x_ddot = (m * g * sin * cos - m * L / 2 * sin * theta_dot**2 + F) / common
#         theta_ddot = (- m * L / 2 * sin * cos * theta_dot**2 + (M + m) * g * sin + F * cos) / (L / 2 * common)

#         states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

#         continuous_dynamic_model = casadi.Function("f", [self.state, self.control], [states_dot], ['x', 'u'], ['x_dot'])

#         return continuous_dynamic_model

#     def make_integrator(self):
#         # Integrator
#         states = casadi.SX.sym("states", self.state_dim)
#         ctrls = casadi.SX.sym("ctrls", self.control_dim)

#         f = self.make_f()
#         ode = f(x=states, u=ctrls)["x_dot"]

#         dae = {"x": states, "p": ctrls, "ode": ode}

#         I = casadi.integrator("I", "cvodes", dae, 0, self.control_sampling_time)

#         return I

#     def animate(self, X, U, t_eval, filename="cart_pole.gif"):
#         # Animation
#         fig = plt.figure(figsize=(12, 6))
#         ax = fig.add_subplot(111)
#         frames = np.arange(0, t_eval.size)
#         fps = 1 / self.control_sampling_time

#         def update_figure(i):
#             x_lim_min = -0.5
#             x_lim_max = 0.5
#             y_lim_min = -1
#             y_lim_max = 1
#             u_scale = 10

#             ax.cla()
#             ax.set_xlim(x_lim_min, x_lim_max)
#             ax.set_ylim(y_lim_min, y_lim_max)
#             ax.set_aspect("equal")
#             ax.set_title(f"t{t_eval[i]: 0.2f}")

#             x, theta, _, _ = X[i]
#             u, = U[i]

#             points = np.array([
#                 [x, x - self.pole_length * np.sin(theta)],
#                 [0, self.pole_length * np.cos(theta)]
#             ])

#             ax.hlines(0, x_lim_min, x_lim_max, colors="black")
#             ax.scatter(*points, color="blue", s=0) # dot size
#             ax.plot(*points, color="blue", lw=8) # pole width
#             ax.arrow(x, 0, u / u_scale, 0, width=0.02, head_width=0.06, head_length=0.12)

#             w = 0.2 # cart width
#             h = 0.1 # cart width
#             rect = patches.Rectangle(xy=(x - w / 2, - h / 2), width=w, height=h, color="black")
#             ax.add_patch(rect)

#         ani = FuncAnimation(fig, update_figure, frames=frames)
#         ani.save(filename, writer="pillow", fps=fps)

# if __name__ == "__main__":
#     cart_pole_nmpc = CartPoleNMPC()
#     visualize = Visualize()

#     cart_pole_nmpc.set_reference_state(casadi.DM([0, 0, 0, 0]))
#     cart_pole_nmpc.set_reference_control(casadi.DM([0]))

#     # # 초기 상태 설정
#     # current_state = np.array([0.2, 0.0, 0.0, 0.0])  # 예: [x, theta, x_dot, theta_dot]

#     current_state = casadi.DM([0, np.pi, 0, 0])

#     actual_state_trajectory, actual_control_trajectory, time_steps = visualize.simulate(current_state, [0, 5])

#     cart_pole_nmpc.visualize(actual_state_trajectory, actual_control_trajectory, time_steps)

#     cart_pole_nmpc.animate(actual_state_trajectory, actual_control_trajectory, time_steps, filename="cart_pole_animation.gif")
