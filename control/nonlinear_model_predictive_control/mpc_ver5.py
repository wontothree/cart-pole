# 원본 코드를 클래스로 만듦
import numpy as np
import casadi
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

class CartPoleNMPC:
    def __init__(self):
        # Linear inverted pendulum parameter
        self.g = 9.81                # Gravitational acceleration (m/s^2)
        self.M = 1                   # Cart mass (kg)
        self.m = 0.2                 # Pole mass (kg)
        self.l = 1                   # Pole length (m)
        self.nu = 1                  # Control variable dimensions
        self.nx = 4                  # State variable dimensions

        # Cost function weights
        self.Q = casadi.diag([2.5, 10, 0.01, 0.01])
        self.Q_f = casadi.diag([2.5, 10, 0.01, 0.01])
        self.R = casadi.diag([0.1])

        # Prediction horizon
        self.T = 1
        self.K = 20
        self.dt = self.T / self.K

        # Constraints
        self.x_lb = [-np.inf, -np.inf, -np.inf, -np.inf]
        self.x_ub = [np.inf, np.inf, np.inf, np.inf]
        self.u_lb = [-15]
        self.u_ub = [15]

        # Target value
        self.x_ref = casadi.DM([0, 0, 0, 0])
        self.u_ref = casadi.DM([0])

        self.total = (self.K + 1) * self.nx + self.K * self.nu

    def make_f(self):
        # State equation
        states = casadi.SX.sym("states", self.nx)
        ctrls = casadi.SX.sym("ctrls", self.nu)

        x = states[0]
        theta = states[1]
        x_dot = states[2]
        theta_dot = states[3]
        F = ctrls[0]

        sin = casadi.sin(theta)
        cos = casadi.cos(theta)
        det = self.M + self.m * sin**2

        x_ddot = (self.m * self.l * sin * theta_dot**2 - self.m * self.g * sin * cos + F) / det
        theta_ddot = (- self.m * self.l * sin * cos * theta_dot**2 + (self.M + self.m) * self.g * sin - F * cos) / (self.l * det)

        states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

        f = casadi.Function("f", [states, ctrls], [states_dot], ['x', 'u'], ['x_dot'])

        return f

    def make_F_RK4(self):
        # RK4 Integrator
        states = casadi.SX.sym("states", self.nx)
        ctrls = casadi.SX.sym("ctrls", self.nu)

        f = self.make_f()

        r1 = f(x=states, u=ctrls)["x_dot"]
        r2 = f(x=states + self.dt * r1 / 2, u=ctrls)["x_dot"]
        r3 = f(x=states + self.dt * r2 / 2, u=ctrls)["x_dot"]
        r4 = f(x=states + self.dt * r3, u=ctrls)["x_dot"]

        states_next = states + self.dt * (r1 + 2 * r2 + 2 * r3 + r4) / 6

        F_RK4 = casadi.Function("F_RK4", [states, ctrls], [states_next], ["x", "u"], ["x_next"])

        return F_RK4

    def make_integrator(self):
        # Integrator
        states = casadi.SX.sym("states", self.nx)
        ctrls = casadi.SX.sym("ctrls", self.nu)

        f = self.make_f()
        ode = f(x=states, u=ctrls)["x_dot"]

        dae = {"x": states, "p": ctrls, "ode": ode}

        I = casadi.integrator("I", "cvodes", dae, 0, self.dt)

        return I

    def compute_state_cost(self, x, u):
        # State cost calculation
        x_diff = x - self.x_ref
        u_diff = u - self.u_ref
        cost = (casadi.dot(self.Q @ x_diff, x_diff)) + casadi.dot(self.R @ u_diff, u_diff) / 2

        return cost

    def compute_terminal_cost(self, x):
        # Terminal cost calculation
        x_diff = x - self.x_ref
        cost = casadi.dot(self.Q_f @ x_diff, x_diff) / 2

        return cost

    def make_nlp(self):
        # Nonlinear programming problem setup
        F_RK4 = self.make_F_RK4()

        U = [casadi.SX.sym(f"u_{k}", self.nu) for k in range(self.K)]
        X = [casadi.SX.sym(f"x_{k}", self.nx) for k in range(self.K + 1)]
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

    def compute_optimal_control(self, S, x_init, x0):
        # Control input calculation
        x_init = x_init.full().ravel().tolist()

        lbx = x_init + self.K * self.x_lb + self.K * self.u_lb
        ubx = x_init + self.K * self.x_ub + self.K * self.u_ub
        lbg = [0] * self.K * self.nx
        ubg = [0] * self.K * self.nx

        res = S(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, x0=x0)

        offset = (self.K + 1) * self.nx
        x0 = res["x"]
        u_opt = x0[offset: offset + self.nu]
        return u_opt, x0

    def run_mpc(self, x_init, t_span=[0, 10]):
        # MPC loop
        S = self.make_nlp()
        t_eval = np.arange(*t_span, self.dt)

        x0 = casadi.DM.zeros(self.total)
        I = self.make_integrator()

        X = [x_init]
        U = []
        x_current = x_init
        for t in t_eval:
            u_opt, x0 = self.compute_optimal_control(S, x_current, x0)
            x_current = I(x0=x_current, p=u_opt)["xf"]
            X.append(x_current)
            U.append(u_opt)

        X.pop()
        X = np.array(X).reshape(t_eval.size, self.nx)
        U = np.array(U).reshape(t_eval.size, self.nu)

        return X, U, t_eval

    def visualize(self, X, U, t_eval):
        # Visualization
        plt.figure(figsize=(12, 4))

        # State plot
        plt.subplot(1, 2, 1)
        for k in range(self.nx):
            plt.plot(t_eval, X[:, k], label=f"x_{k}")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("state")

        # Control plot
        plt.subplot(1, 2, 2)
        for k in range(self.nu):
            plt.step(t_eval, U[:, k], linestyle="--", label=f"u_{k}")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Control")

        plt.show()

    def animate(self, X, U, t_eval, filename="cart_pole.gif"):
        # Animation
        fig = plt.figure(figsize=(12, 6))
        ax = fig.add_subplot(111)
        frames = np.arange(0, t_eval.size)
        fps = 1 / self.dt

        def update_figure(i):
            x_lim_min = -4
            x_lim_max = 4
            y_lim_min = -2
            y_lim_max = 2
            u_scale = 15

            ax.cla()
            ax.set_xlim(x_lim_min, x_lim_max)
            ax.set_ylim(y_lim_min, y_lim_max)
            ax.set_aspect("equal")
            ax.set_title(f"t{t_eval[i]: 0.2f}")

            x, theta, _, _ = X[i]
            u, = U[i]

            points = np.array([
                [x, x - self.l * np.sin(theta)],
                [0, self.l * np.cos(theta)]
            ])

            ax.hlines(0, x_lim_min, x_lim_max, colors="black")
            ax.scatter(*points, color="blue", s=50)
            ax.plot(*points, color="blue", lw=2)
            ax.arrow(x, 0, u / u_scale, 0, width=0.02, head_width=0.06, head_length=0.12)

            w = 0.2
            h = 0.1
            rect = patches.Rectangle(xy=(x - w / 2, - h / 2), width=w, height=h, color="black")
            ax.add_patch(rect)

        ani = FuncAnimation(fig, update_figure, frames=frames)
        ani.save(filename, writer="pillow", fps=fps)


# Usage
cart_pole_nmpc = CartPoleNMPC()
x_init = casadi.DM([0, np.pi, 0, 0])  # Initial value
X, U, t_eval = cart_pole_nmpc.run_mpc(x_init)

# Visualize the results
cart_pole_nmpc.visualize(X, U, t_eval)

# Create animation
cart_pole_nmpc.animate(X, U, t_eval)
