import numpy as np
import casadi
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

class CartPoleMPC:
    def __init__(self):
        # cart pole parameters
        self.grivitation_accel = 9.81
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

        # self.total = (K + 1) * nx + K * nu
        # self.S = self.make_nlp()
        # self.I = self.make_integrator()

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
        cart_accel = ctrls[0]

        sin = casadi.sin(pole_ang)
        cos = casadi.cos(pole_ang)
        det = self.M + self.m * sin**2

        x_ddot = (self.m * self.L * sin * theta_dot**2 - self.m * self.g * sin * cos + F) / det
        theta_ddot = (- self.m * self.L * sin * cos * theta_dot**2 + (self.M + self.m) * self.g * sin - F * cos) / (self.L * det)

        states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

        f = casadi.Function("f", [states, ctrls], [states_dot], ['x', 'u'], ['x_dot'])
        return f