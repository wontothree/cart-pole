import numpy as np
import casadi

from mpc import MPC

class CartPoleNMPC(MPC):
    def __init__(self):
        super().__init__(state_dim=4, control_dim=1, prediction_horizon=50, control_sampling_time=0.1)

        # System parameters
        self.gravity = 9.81
        self.cart_mass = 0.123
        self.pole_mass = 0.089
        self.pole_length = 0.4

        # State and control variables
        self.state = casadi.SX.sym("state", self.state_dim)
        self.ctrl = casadi.SX.sym("ctrl", self.control_dim)

        # Cost function weights
        self.Q = casadi.diag([1.5, 10, 0.0001, 0.0001])
        self.Q_f = casadi.diag([1.5, 10, 0.0001, 0.0001])
        self.R = casadi.diag([0.1])

        # Constraints
        self.state_lower_bound = [-0.36, -np.inf, -np.inf, -np.inf]
        self.state_upper_bound = [0.36, np.inf, np.inf, np.inf]
        self.ctrl_lower_bound = [-1]
        self.ctrl_upper_bound = [1]

        # number of optimization variables
        self.total_variables = (self.prediction_horizon + 1) * self.state_dim + self.prediction_horizon * self.control_dim

    def define_dynamic_model(self):
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

        # discretize
        r1 = f(x=self.state, u=self.ctrl)["x_dot"]
        r2 = f(x=self.state + self.control_sampling_time * r1 / 2, u=self.ctrl)["x_dot"]
        r3 = f(x=self.state + self.control_sampling_time * r2 / 2, u=self.ctrl)["x_dot"]
        r4 = f(x=self.state + self.control_sampling_time * r3, u=self.ctrl)["x_dot"]

        states_next = self.state + self.control_sampling_time * (r1 + 2 * r2 + 2 * r3 + r4) / 6

        F_Rprediction_horizon4 = casadi.Function("F_Rprediction_horizon4", [self.state, self.ctrl], [states_next], ["x", "u"], ["x_next"])

        return F_Rprediction_horizon4

    def define_cost_function(self):
        pass

    def define_constraints(self):
        pass

    def solve(self):
        pass
