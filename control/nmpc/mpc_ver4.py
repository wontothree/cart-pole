import numpy as np
import casadi
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

class CartPoleMPC:
    def __init__(self):
        # cart pole parameters
        self.gravity = 9.81
        self.cart_mass = 0.123                  # mass of cart (kg)
        self.pole_mass = 0.089                  # mass of pole (kg)
        self.pole_inertia = 0.004746            # interia of pole (kg m^2)
        self.pole_length = 0.4                  # length of pole (m)

        self.states_dim = 4                     # dimension of control input vector
        self.ctrls_dim = 1                      # dimension of state variable vector     

        self.T = 1
        self.K = 20
        self.dt = 1 / 20

        # Target value
        self.states_ref = casadi.DM([0, 0, 0, 0])
        self.ctrls_ref = casadi.DM([0])

        # Cost function weights
        self.Q = casadi.diag([2.5, 10, 0.01, 0.01])
        self.Q_f = casadi.diag([2.5, 10, 0.01, 0.01])
        self.R = casadi.diag([0.1])

        # Constraints
        self.states_lower_bound = [-0.36, -np.inf, -np.inf, -np.inf]
        self.states_upper_bound = [0.36, np.inf, np.inf, np.inf]
        self.ctrls_lower_bound = [-2]
        self.ctrls_upper_bound = [2]

        self.total = (20 + 1) * 1 + 20 * 4
        self.S = self.solve_nlp()
        self.I = self.update_next_state()

    def formulate_state_equations(self):
        # state variable vector and control input variable vector
        states = casadi.SX.sym("states", self.states_dim)
        ctrls = casadi.SX.sym("ctrls", self.ctrls_dim)

        # state variables
        cart_pos = states[0]                    # position of cart
        cart_vel = states[1]                    # velocity of cart
        pole_ang = states[2]                    # angle of pole
        pole_ang_vel = states[3]                # angular velocity of pole

        # control input variable
        power = ctrls[0]

        sin = casadi.sin(pole_ang)
        cos = casadi.cos(pole_ang)

        det = self.cart_mass + self.pole_mass * sin**2
        
        cart_pos_dot = cart_vel
        cart_vel_dot = (self.pole_mass* self.pole_length * sin * pole_ang_vel**2 - self.pole_mass * self.gravity * sin * cos + power) / det
        pole_ang_dot = pole_ang_vel
        pole_ang_vel_dot = (- self.pole_mass* self.pole_length * sin * cos * pole_ang_vel**2 + (self.cart_mass + self.pole_mass) * self.gravity * sin - power * cos) / (self.pole_length * det)

        # derivation of state variables
        states_dot = casadi.vertcat(cart_pos_dot, cart_vel_dot, pole_ang_dot, pole_ang_vel_dot)

        state_equation = casadi.Function("f", [states, ctrls], [states_dot], ['x', 'u'], ['x_dot'])

        return state_equation
        
    def predict_next_state(self):
        states = casadi.SX.sym("states", self.states_dim)
        ctrls = casadi.SX.sym("ctrls", self.ctrls_dim)

        f = self.formulate_state_equations()

        r1 = f(x=states, u=ctrls)["x_dot"]
        r2 = f(x=states + self.dt * r1 / 2, u=ctrls)["x_dot"]
        r3 = f(x=states + self.dt * r2 / 2, u=ctrls)["x_dot"]
        r4 = f(x=states + self.dt * r3, u=ctrls)["x_dot"]

        states_next = states + self.dt * (r1 + 2 * r2 + 2 * r3 + r4) / 6

        F_RK4 = casadi.Function("F_RK4", [states, ctrls], [states_next], ["x", "u"], ["x_next"])

        return F_RK4

    def update_next_state(self):
        states = casadi.SX.sym("states", self.states_dim)
        ctrls = casadi.SX.sym("ctrls", self.ctrls_dim)

        f = self.formulate_state_equations()
        ode = f(x = states, u = ctrls)["x_dot"]

        dae = {"x" : states, "p" : ctrls, "ode" : ode}
        next_states = casadi.integrator("I", "cvodes", dae, 0, self.dt)
        return next_states


    def compute_state_cost(self, states, ctrls):
        states_diff = states - self.states_ref
        ctrls_diff = ctrls - self.ctrls_ref
        state_cost = (casadi.dot(self.Q @ states_diff, states_diff)) + casadi.dot(self.R @ ctrls_diff, ctrls_diff) / 2
        return state_cost

    def compute_terminal_cost(self, states):
        states_diff = states - self.states_ref
        terminal_cost = casadi.dot(self.Q_f @ states_diff, states_diff) / 2
        return terminal_cost
    
    def solve_nlp(self):
        F_RK4 = self.predict_next_state()

        U = [casadi.SX.sym(f"u_{k}", self.ctrls_dim) for k in range(self.K)]
        X = [casadi.SX.sym(f"x_{k}", self.states_dim) for k in range(self.K + 1)]
        G = []
        J = 0

        for k in range(self.K):
            J += self.compute_state_cost(X[k], U[k]) * self.dt
            eq = X[k + 1] - F_RK4(x=X[k], u=U[k])["x_next"]
            G.append(eq)

        J += self.compute_terminal_cost(X[-1])

        option = {'print_time': False, 'ipopt': {'max_iter': 10, 'print_level': 0}}
        nlp = {"x": casadi.vertcat(*X, *U), "f": J, "g": casadi.vertcat(*G)}
        S = casadi.nlpsol("S", "ipopt", nlp, option)
        return S

    def compute_optimal_control(self, x_init, x0):
        x_init = x_init.full().ravel().tolist()

        lbx = x_init + self.K * self.states_lower_bound + self.K * self.ctrls_lower_bound
        ubx = x_init + self.K * self.states_upper_bound + self.K * self.ctrls_upper_bound
        lbg = [0] * self.K * self.states_dim
        ubg = [0] * self.K * self.states_dim

        res = self.S(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, x0=x0)
        offset = (self.K + 1) * self.states_dim
        x0 = res["x"]
        u_opt = x0[offset:offset + self.ctrls_dim]
        return u_opt, x0
    

    def run_mpc(self, t_span):
        t_eval = np.arange(*t_span, self.dt)
        x_init = casadi.DM([0, np.pi, 0, 0])  # initial value
        x0 = casadi.DM.zeros(self.total)

        X = [x_init]
        U = []
        x_current = x_init

        for t in t_eval:
            u_opt, x0 = self.compute_optimal_control(x_current, x0)
            x_current = self.I(x0=x_current, p=u_opt)["xf"]
            X.append(x_current)
            U.append(u_opt)

        X.pop()
        X = np.array(X).reshape(t_eval.size, self.nx)
        U = np.array(U).reshape(t_eval.size, self.nu)

        self.visualize_results(t_eval, X, U)
        self.create_animation(t_eval, X, U)

    def visualize_results(self, t_eval, X, U):
        plt.figure(figsize=(12, 4))
        plt.subplot(1, 2, 1)

        for k in range(self.nx):
            plt.plot(t_eval, X[:, k], label=f"x_{k}")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("State")

        plt.subplot(1, 2, 2)
        for k in range(self.nu):
            plt.step(t_eval, U[:, k], linestyle="--", label=f"u_{k}")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Control")

        plt.show()

    def create_animation(self, t_eval, X, U):
        fig, ax = plt.subplots()

        ax.set_xlim(-1, 1)
        ax.set_ylim(-0.5, 1.5)
        ax.set_aspect('equal')

        cart_width = 0.2
        cart_height = 0.1
        pole_length = self.pole_length

        # Create the cart and pole objects
        cart = patches.Rectangle((-cart_width / 2, 0), cart_width, cart_height, fc='b')
        pole, = ax.plot([], [], lw=2)

        def init():
            ax.add_patch(cart)
            pole.set_data([], [])
            return cart, pole

        def update(frame):
            # Cart position (x)
            cart_pos = X[frame, 0]

            # Pole angle (theta)
            pole_ang = X[frame, 2]

            # Update cart position
            cart.set_xy([cart_pos - cart_width / 2, 0])

            # Calculate the new pole position
            pole_x = [cart_pos, cart_pos + pole_length * np.sin(pole_ang)]
            pole_y = [cart_height / 2, cart_height / 2 - pole_length * np.cos(pole_ang)]
            pole.set_data(pole_x, pole_y)

            return cart, pole

        ani = FuncAnimation(fig, update, frames=len(t_eval), init_func=init, blit=True, interval=50)
        plt.show()


    # def create_animation(self, t_eval, X, U):
    #     fig = plt.figure(figsize=(12, 6))
    #     ax = fig.add_subplot(111)
    #     frames = np.arange(0, t_eval.size)
    #     fps = 1 / self.dt

    #     def update_figure(i):
    #         x_lim_min = -4
    #         x_lim_max = 4
    #         y_lim_min = -2
    #         y_lim_max = 2
    #         u_scale = 15

    #         ax.cla()
    #         ax.set_xlim(x_lim_min, x_lim_max)
    #         ax.set_ylim(y_lim_min, y_lim_max)
    #         ax.set_aspect("equal")
    #         ax.set_title(f"t{t_eval[i]:0.2f}")

    #         x, theta, _, _ = X[i]
    #         u, = U[i]

    #         points = np.array([
    #             [x, x - self.L * np.sin(theta)],
    #             [0, self.L * np.cos(theta)]
    #         ])

    #         ax.plot(points[:, 0], points[:, 1], 'ro-', markersize=8, label="Pendulum")

    #         pendulum = patches.Polygon(points, closed=False, fill=False, edgecolor="blue", linewidth=2)
    #         ax.add_patch(pendulum)

    #         ax.plot(x, 0, 'bo', markersize=8)
    #         ax.plot(x - self.L * np.sin(theta), 0, 'ro', markersize=8)
    #         ax.plot([x, x - self.L * np.sin(theta)], [0, 0], 'k--')

    #         ax.annotate(f'u={u:.2f}', (x - self.L * np.sin(theta), 0), textcoords="offset points", xytext=(0,10), ha='center')

    #     ani = FuncAnimation(fig, update_figure, frames=frames, interval=1000 / fps, repeat=False)
    #     plt.show()

# Usage
if __name__ == "__main__":
    mpc = CartPoleMPC()
    mpc.run_mpc(t_span=(0, 10))