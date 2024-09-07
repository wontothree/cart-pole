# refactoring
import numpy as np
import casadi

class CartPoleNMPC:
    def __init__(self):
        # constant parameter
        self.gravity = 9.81                         # Gravitational acceleration (m/s^2)
        self.cart_mass = 0.123                      # Cart mass (prediction_horizong)
        self.pole_mass = 0.089                      # Pole mass (prediction_horizong)
        self.pole_length = 0.4                      # Pole length (m)

        # dimension of state and control
        self.state_dim = 4                          # State variable dimensions
        self.ctrl_dim = 1                           # Control variable dimensions

        self.state = casadi.SX.sym("state", self.state_dim)
        self.ctrl = casadi.SX.sym("ctrl", self.ctrl_dim)

        # cost function weights
        self.Q = casadi.diag([1.5, 10, 0.0001, 0.0001])
        self.Q_f = casadi.diag([1.5, 10, 0.0001, 0.0001])
        self.R = casadi.diag([0.1])

        # prediction prediction_horizon and control sampling time
        self.prediction_horizon = 50                                                                   # prediction_Horizon
        self.ctrl_sampling_time = 0.1                                                                  # Control Sampling Time
        self.total_prediction_time = self.prediction_horizon * self.ctrl_sampling_time                 # Prediction Time

        # constraints
        self.state_lower_bound = [-0.36, -np.inf, -np.inf, -np.inf]
        self.state_lower_bound = [0.36, np.inf, np.inf, np.inf]
        self.ctrl_lower_bound = [-1]
        self.ctrl_upper_bound = [1]

        # target value
        self.x_ref = casadi.DM([0, 0, 0, 0])
        self.u_ref = casadi.DM([0])

        self.total_variables = (self.prediction_horizon + 1) * self.state_dim + self.prediction_horizon * self.ctrl_dim

    def calc_continuous_dynamics(self):
        # constant parameter
        g = self.gravity
        M = self.cart_mass
        m = self.pole_mass
        L = self.pole_length

        # state variables
        x = self.state[0]
        theta = self.state[1]
        x_dot = self.state[2]
        theta_dot = self.state[3]

        # control input
        F = self.ctrl[0]

        sin = casadi.sin(theta)
        cos = casadi.cos(theta)
        det = self.M + self.m * sin**2

        # dynamics
        x_ddot = (m * L * sin * theta_dot**2 + m * g * sin * cos + F) / det
        theta_ddot = (m * L * sin * cos * theta_dot**2 +
                      (M + m) * g * sin + F * cos) / (L * det)

        states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

        f = casadi.Function("f", [self.state, self.ctrl], [states_dot], ['x', 'u'], ['x_dot'])

        return f

    def calc_discrete_dynamics(self):
        f = self.calc_continuous_dynamics()

        r1 = f(x=self.state, u=self.ctrl)["x_dot"]
        r2 = f(x=self.state + self.ctrl_sampling_time * r1 / 2, u=self.ctrl)["x_dot"]
        r3 = f(x=self.state + self.ctrl_sampling_time * r2 / 2, u=self.ctrl)["x_dot"]
        r4 = f(x=self.state + self.ctrl_sampling_time * r3, u=self.ctrl)["x_dot"]

        states_next = self.state + self.ctrl_sampling_time * (r1 + 2 * r2 + 2 * r3 + r4) / 6

        F_Rprediction_horizon4 = casadi.Function("F_Rprediction_horizon4", [self.state, self.ctrl], [states_next], ["x", "u"], ["x_next"])

        return F_Rprediction_horizon4

    # state cost
    def calc_state_cost(self, x, u):
        x_diff = x - self.x_ref
        u_diff = u - self.u_ref
        state_cost = (casadi.dot(self.Q @ x_diff, x_diff) + \
            casadi.dot(self.R @ u_diff, u_diff)) / 2

        return state_cost

    # terminal cost
    def calc_terminal_cost(self, x):
        x_diff = x - self.x_ref
        terminal_cost = casadi.dot(self.Q_f @ x_diff, x_diff) / 2

        return terminal_cost

    # solver
    def solve_nip(self):
        # Nonlinear programming problem setup
        F_Rprediction_horizon4 = self.calc_discrete_dynamics()

        U = [casadi.SX.sym(f"u_{prediction_horizon}", self.nu) for prediction_horizon in range(self.prediction_horizon)]
        X = [casadi.SX.sym(f"x_{prediction_horizon}", self.nx) for prediction_horizon in range(self.prediction_horizon + 1)]
        G = []

        J = 0

        for prediction_horizon in range(self.prediction_horizon):
            J += self.calc_state_cost(X[prediction_horizon], U[prediction_horizon]) * self.ctrl_sampling_time
            eq = X[prediction_horizon + 1] - F_Rprediction_horizon4(x=X[prediction_horizon], u=U[prediction_horizon])["x_next"]
            G.append(eq)

        J += self.calc_terminal_cost(X[-1])

        option = {'print_time': False, 'ipopt': {
            'max_iter': 10, 'print_level': 0}}
        nlp = {"x": casadi.vertcat(*X, *U), "f": J, "g": casadi.vertcat(*G)}
        S = casadi.nlpsol("S", "ipopt", nlp, option)

        return S

    def calc_optimal_control(self, S, init_state, x0):
        # Control input calculation
        init_state = init_state.full().ravel().tolist()

        # constraints
        lbx = init_state + self.prediction_horizon * self.state_lower_bound + self.prediction_horizon * self.ctrl_lower_bound
        ubx = init_state + self.prediction_horizon * self.state_lower_bound + self.prediction_horizon * self.ctrl_upper_bound
        lbg = [0] * self.prediction_horizon * self.state_dim
        ubg = [0