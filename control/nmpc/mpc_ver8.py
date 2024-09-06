# refactoring
import numpy as np
import casadi


class CartPoleNMPC:
    def __init__(self):
        # constant parameter
        # Gravitational acceleration (m/s^2)
        self.g = 9.81
        self.M = 0.123                      # Cart mass (kg)
        self.m = 0.089                      # Pole mass (kg)
        self.l = 0.4                        # Pole length (m)

        # dimension of state and control
        self.state_dim = 4                  # State variable dimensions
        self.ctrl_dim = 1                   # Control variable dimensions

        self.state = casadi.SX.sym("state", self.state_dim)
        self.ctrl = casadi.SX.sym("ctrl", self.ctrl_dim)

        # cost function weights
        self.Q = casadi.diag([1.5, 10, 0.0001, 0.0001])
        self.Q_f = casadi.diag([1.5, 10, 0.0001, 0.0001])
        self.R = casadi.diag([0.1])

        # prediction horizon and control sampling time
        self.T = 1
        self.K = 20
        self.dt = self.T / self.K

        # constraints
        self.state_lower_bound = [-0.36, -np.inf, -np.inf, -np.inf]
        self.state_lower_bound = [0.36, np.inf, np.inf, np.inf]
        self.ctrl_lower_bound = [-1]
        self.ctrl_upper_bound = [1]

        # target value
        self.x_ref = casadi.DM([0, 0, 0, 0])
        self.u_ref = casadi.DM([0])

        self.total = (self.K + 1) * self.state_dim + self.K * self.ctrl_dim

    def calc_continuous_dynamics(self):
        g = self.g
        M = self.M
        m = self.m
        l = self.l

        x = self.state[0]
        theta = self.state[1]
        x_dot = self.state[2]
        theta_dot = self.state[3]
        F = self.ctrl[0]

        sin = casadi.sin(theta)
        cos = casadi.cos(theta)
        det = self.M + self.m * sin**2

        x_ddot = (m * l * sin * theta_dot**2 + m * g * sin * cos + F) / det
        theta_ddot = (m * l * sin * cos * theta_dot**2 +
                      (M + m) * g * sin + F * cos) / (l * det)

        states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

        f = casadi.Function("f", [self.state, self.ctrl], [states_dot], ['x', 'u'], ['x_dot'])

        return f

    def calc_discrete_dynamics(self):
        f = self.calc_continuous_dynamics()

        r1 = f(x=self.state, u=self.ctrl)["x_dot"]
        r2 = f(x=self.state + self.dt * r1 / 2, u=self.ctrl)["x_dot"]
        r3 = f(x=self.state + self.dt * r2 / 2, u=self.ctrl)["x_dot"]
        r4 = f(x=self.state + self.dt * r3, u=self.ctrl)["x_dot"]

        states_next = self.state + self.dt * (r1 + 2 * r2 + 2 * r3 + r4) / 6

        F_RK4 = casadi.Function("F_RK4", [self.state, self.ctrl], [
                                states_next], ["x", "u"], ["x_next"])

        return F_RK4

    def update_next_state(self):

        f = self.calc_continuous_dynamics()
        ode = f(x=self.state, u=self.ctrl)["x_dot"]

        dae = {"x": self.state, "p": self.ctrl, "ode": ode}

        I = casadi.integrator("I", "cvodes", dae, 0, self.dt)

        return I

    def calc_state_cost(self, x, u):
        x_diff = x - self.x_ref
        u_diff = u - self.u_ref
        state_cost = (casadi.dot(self.Q @ x_diff, x_diff)) + \
            casadi.dot(self.R @ u_diff, u_diff) / 2

        return state_cost

    def calc_terminal_cost(self, x):
        x_diff = x - self.x_ref
        terminal_cost = casadi.dot(self.Q_f @ x_diff, x_diff) / 2

        return terminal_cost

    def solve_nip(self):
        # Nonlinear programming problem setup
        F_RK4 = self.calc_discrete_dynamics()

        U = [casadi.SX.sym(f"u_{k}", self.nu) for k in range(self.K)]
        X = [casadi.SX.sym(f"x_{k}", self.nx) for k in range(self.K + 1)]
        G = []

        J = 0

        for k in range(self.K):
            J += self.calc_state_cost(X[k], U[k]) * self.dt
            eq = X[k + 1] - F_RK4(x=X[k], u=U[k])["x_next"]
            G.append(eq)

        J += self.calc_terminal_cost(X[-1])

        option = {'print_time': False, 'ipopt': {
            'max_iter': 10, 'print_level': 0}}
        nlp = {"x": casadi.vertcat(*X, *U), "f": J, "g": casadi.vertcat(*G)}
        S = casadi.nlpsol("S", "ipopt", nlp, option)

        return S

    def calc_optimal_control(self, S, x_init, x0):
        # Control input calculation
        x_init = x_init.full().ravel().tolist()

        lbx = x_init + self.K * self.states_lower_bound + self.K * self.u_lb
        ubx = x_init + self.K * self.x_ub + self.K * self.u_ub
        lbg = [0] * self.K * self.nx
        ubg = [0] * self.K * self.nx

        res = S(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, x0=x0)

        offset = (self.K + 1) * self.nx
        x0 = res["x"]
        u_opt = x0[offset: offset + self.nu]
        return u_opt, x0

    def run_mpc(self, x_init, t_span=[0, 15]):
        # MPC loop
        S = self.solve_nip()
        t_eval = np.arange(*t_span, self.dt)

        x0 = casadi.DM.zeros(self.total)
        I = self.update_next_state()

        X = [x_init]
        U = []
        x_current = x_init
        for t in t_eval:
            u_opt, x0 = self.calc_optimal_control(S, x_current, x0)
            x_current = I(x0=x_current, p=u_opt)["xf"]
            X.append(x_current)
            U.append(u_opt)

        X.pop()
        X = np.array(X).reshape(t_eval.size, self.state_dim)
        U = np.array(U).reshape(t_eval.size, self.ctrl_dim)

        return X, U, t_eval


if __name__ == "__main__":
    # Usage
    cart_pole_nmpc = CartPoleNMPC()
    x_init = casadi.DM([0, np.pi, 1, 0])  # Initial value
    X, U, t_eval = cart_pole_nmpc.run_mpc(x_init)

    # Visualize the results
    cart_pole_nmpc.visualize(X, U, t_eval)

    # Create animation
    cart_pole_nmpc.animate(X, U, t_eval)