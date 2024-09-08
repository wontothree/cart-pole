import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

import casadi

from mpc import MPC

class CartPoleNMPC(MPC):
    def __init__(self):
        super().__init__(state_dim=4, control_dim=1, prediction_horizon=2, control_sampling_time=0.1)

        # System parameters
        self.gravity = 9.81
        self.cart_mass = 0.123
        self.pole_mass = 0.089
        self.pole_length = 0.4

        # State and control variables
        self.state = casadi.SX.sym("state", self.state_dim)
        self.control = casadi.SX.sym("control", self.control_dim)

        # Cost function weights
        self.Q = casadi.diag([1, 10, 0.0001, 0.0001])
        self.Q_f = casadi.diag([1, 10, 0.0001, 0.0001])
        self.R = casadi.diag([0.1])

        # Constraints
        self.state_lower_bound = [-0.36, -np.inf, -np.inf, -np.inf]
        self.state_upper_bound = [0.36, np.inf, np.inf, np.inf]
        self.control_lower_bound = [-1]
        self.control_upper_bound = [1]

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

        # control input variable
        F = self.control[0]

        sin = casadi.sin(theta)
        cos = casadi.cos(theta)
        common = M + m * sin**2

        # dynamics
        x_ddot = (m * g * sin * cos - m * L / 2 * sin * theta_dot**2 + F) / common
        theta_ddot = (- m * L / 2 * sin * cos * theta_dot**2 + (M + m) * g * sin + F * cos) / (L / 2 * common)

        states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

        continuous_dynamic_model = casadi.Function("f", [self.state, self.control], [states_dot], ['x', 'u'], ['x_dot'])

        # discretize
        r1 = continuous_dynamic_model(x=self.state, u=self.control)["x_dot"]
        r2 = continuous_dynamic_model(x=self.state + self.control_sampling_time * r1 / 2, u=self.control)["x_dot"]
        r3 = continuous_dynamic_model(x=self.state + self.control_sampling_time * r2 / 2, u=self.control)["x_dot"]
        r4 = continuous_dynamic_model(x=self.state + self.control_sampling_time * r3, u=self.control)["x_dot"]

        states_next = self.state + self.control_sampling_time * (r1 + 2 * r2 + 2 * r3 + r4) / 6

        discretized_dynamic_model = casadi.Function("discretied_dynamic_model", [self.state, self.control], [states_next], ["x", "u"], ["x_next"])

        return discretized_dynamic_model
    
    def define_constraint(self, state_trajectory, control_trajectory, initial_state):
        discretized_dynamic_model = self.define_dynamic_model()
        # equality constraint
        equality_constraint_lower_bound = [0] * (self.prediction_horizon * self.state_dim)
        equality_constraint_upper_bound = [0] * (self.prediction_horizon * self.state_dim)
        equality_constraint = []
        for step in range(self.prediction_horizon):
            predicted_next_state = discretized_dynamic_model(x = state_trajectory[step], u = control_trajectory[step])["x_next"]
            constraint = state_trajectory[step  + 1] - predicted_next_state
            equality_constraint.append(constraint)
        
        # inequality constraint
        initial_state = initial_state.full().ravel().tolist()
        optimization_variable_lower_bound = initial_state + self.prediction_horizon * self.state_lower_bound + self.prediction_horizon * self.control_lower_bound
        optimization_variable_upper_bound = initial_state + self.prediction_horizon * self.state_upper_bound + self.prediction_horizon * self.control_upper_bound

        return equality_constraint, equality_constraint_lower_bound, equality_constraint_upper_bound, optimization_variable_lower_bound, optimization_variable_upper_bound

    # cost for one state and control input
    def define_cost_function(self, state_trajectory, control_trajectory):
        """
        - state_trajectory : vector having dimension (prediction_horizon + 1)
        - control_trajectory : vector having dimention (prediction_horizon)
        """
        cost = 0
        state_cost = 0
        terminal_cost = 0

        # state cost
        for step in range(self.prediction_horizon):
            state_error = state_trajectory[step] - self.reference_state
            control_error = control_trajectory[step] - self.reference_control

            state_step_cost = (casadi.dot(self.Q @ state_error, state_error) + casadi.dot(self.R @ control_error, control_error)) / 2

            state_cost += state_step_cost * self.control_sampling_time
        
        # terminal cost
        state_error = state_trajectory[-1] - self.reference_state
        terminal_cost = casadi.dot(self.Q_f @ state_error, state_error)

        cost = state_cost + terminal_cost

        return cost


    def solve(self):
        state_trajectory = [casadi.SX.sym(f"x_{k}", self.state_dim) for k in range(self.prediction_horizon + 1)]
        control_trajectory = [casadi.SX.sym(f"u_{k}", self.control_dim) for k in range(self.prediction_horizon)]

        # cost function
        cost_function = self.define_cost_function(state_trajectory, control_trajectory)

        # equality constraint and inequality constraint
        equality_constraint, equality_constraint_lower_bound, equality_constraint_upper_bound, optimization_variable_lower_bound, optimization_variable_upper_bound = \
            self.define_constraint(state_trajectory, control_trajectory, self.initial_state)
        
        # set up the NLP problem
        nonlinear_programming_problem = {
            "x": casadi.vertcat(*state_trajectory, *control_trajectory),
            "f": cost_function,
            "g": casadi.vertcat(*equality_constraint)
        }

        # solver options
        options = {
            "print_time": True,
            "ipopt": {
                "max_iter": 1000,
                "print_level": 1
            }
        }

        # create the solver
        solver = casadi.nlpsol("solver", "ipopt", nonlinear_programming_problem, options)


        initial_guess = casadi.DM.zeros(self.total_variables)

        # solve the optimization problem
        optimization_problem_solving_result = solver(
            x0 = initial_guess,
            lbx = optimization_variable_lower_bound,
            ubx = optimization_variable_upper_bound,
            lbg = equality_constraint_lower_bound,
            ubg = equality_constraint_upper_bound
        )

        # extract the optimal state and control trajectory
        optimization_problem_solution = optimization_problem_solving_result["x"]
        optimal_state_trajectory = optimization_problem_solution[:(self.prediction_horizon + 1) * self.state_dim]
        optimal_control_trajectory = optimization_problem_solution[(self.prediction_horizon + 1) * self.control_dim]

        return optimal_state_trajectory, optimal_control_trajectory

    # visualize
    # def run_simulation(self, cart_pole_nmpc, x_init, t_span=[0, 15]):
    #     # MPC 루프 설정
    #     dt = self.control_sampling_time
    #     t_eval = np.arange(*t_span, dt)

    #     # 상태와 제어 입력 초기화
    #     X = [x_init]
    #     U = []

    #     # 시뮬레이션 루프
    #     for t in t_eval:
    #         self.set_initial_state(casadi.DM(x_init))
    #         optimal_state_trajectory, optimal_control_trajectory = cart_pole_nmpc.solve()
            
    #         u_opt = optimal_control_trajectory[0]
    #         X.append(optimal_state_trajectory[:cart_pole_nmpc.state_dim].full().ravel())
    #         U.append(u_opt.full().ravel())
            
    #         x_init = X[-1]  # 업데이트된 상태를 다음 단계의 초기 상태로 설정

    #     X = np.array(X)
    #     U = np.array(U)

    #     return X, U, t_eval

    # def visualize(self, X, U, t_eval):
    #     # 시각화
    #     plt.figure(figsize=(12, 4))

    #     # 상태 변수 플롯
    #     plt.subplot(1, 2, 1)
    #     for k in range(X.shape[1]):
    #         plt.plot(t_eval, X[:, k], label=f"x_{k}")
    #     plt.legend()
    #     plt.xlabel("Time")
    #     plt.ylabel("State")

    #     # 제어 입력 플롯
    #     plt.subplot(1, 2, 2)
    #     for k in range(U.shape[1]):
    #         plt.step(t_eval[:-1], U[:, k], linestyle="--", label=f"u_{k}")
    #     plt.legend()
    #     plt.xlabel("Time")
    #     plt.ylabel("Control")

    #     plt.show()

    def run_simulation(self, cart_pole_nmpc, x_init, t_span=[0, 15]):
        # MPC 루프 설정
        dt = self.control_sampling_time
        t_eval = np.arange(t_span[0], t_span[1] + dt, dt)  # + dt 추가

        # 상태와 제어 입력 초기화
        X = [x_init]
        U = []

        # 시뮬레이션 루프
        for t in t_eval[:-1]:  # 마지막 t_eval에 대한 시뮬레이션은 필요 없음
            self.set_initial_state(casadi.DM(x_init))
            optimal_state_trajectory, optimal_control_trajectory = cart_pole_nmpc.solve()
            
            u_opt = optimal_control_trajectory[0]
            X.append(optimal_state_trajectory[:cart_pole_nmpc.state_dim].full().ravel())
            U.append(u_opt.full().ravel())
            
            x_init = X[-1]  # 업데이트된 상태를 다음 단계의 초기 상태로 설정

        X = np.array(X)
        U = np.array(U)

        return X, U, t_eval

    def visualize(self, X, U, t_eval):
        # 시각화
        plt.figure(figsize=(12, 4))

        # 상태 변수 플롯
        plt.subplot(1, 2, 1)
        for k in range(X.shape[1]):
            plt.plot(t_eval[:X.shape[0]], X[:, k], label=f"x_{k}")  # t_eval 길이에 맞게 조정
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("State")

        # 제어 입력 플롯
        plt.subplot(1, 2, 2)
        for k in range(U.shape[1]):
            plt.step(t_eval[:-1], U[:, k], linestyle="--", label=f"u_{k}")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Control")

        plt.show()


    # def animate(X, U, t_eval, filename="cart_pole.gif"):
    #     # 애니메이션
    #     fig = plt.figure(figsize=(12, 6))
    #     ax = fig.add_subplot(111)
    #     frames = np.arange(0, len(t_eval))
    #     fps = 1 / (t_eval[1] - t_eval[0])  # Frame per second

    #     def update_figure(i):
    #         x_lim_min = -1
    #         x_lim_max = 1
    #         y_lim_min = -0.5
    #         y_lim_max = 0.5
    #         u_scale = 10

    #         ax.cla()
    #         ax.set_xlim(x_lim_min, x_lim_max)
    #         ax.set_ylim(y_lim_min, y_lim_max)
    #         ax.set_aspect("equal")
    #         ax.set_title(f"t={t_eval[i]: 0.2f}")

    #         x, theta, _, _ = X[i]
    #         u = U[i]

    #         points = np.array([
    #             [x, x - cart_pole_nmpc.pole_length * np.sin(theta)],
    #             [0, cart_pole_nmpc.pole_length * np.cos(theta)]
    #         ])

    #         ax.hlines(0, x_lim_min, x_lim_max, colors="black")
    #         ax.scatter(*points, color="blue", s=50)
    #         ax.plot(*points, color="blue", lw=2)
    #         ax.arrow(x, 0, u / u_scale, 0, width=0.02, head_width=0.06, head_length=0.12)

    #         w = 0.2
    #         h = 0.1
    #         rect = patches.Rectangle(xy=(x - w / 2, - h / 2), width=w, height=h, color="black")
    #         ax.add_patch(rect)

    #     ani = FuncAnimation(fig, update_figure, frames=frames)
    #     ani.save(filename, writer="pillow", fps=fps)



if __name__ == "__main__":
    cart_pole_nmpc = CartPoleNMPC()

    cart_pole_nmpc.set_reference_state(casadi.DM([0, 0, 0, 0]))
    cart_pole_nmpc.set_reference_control(casadi.DM([0]))

    # 초기 상태 설정
    x_init = np.array([0.0, 0.0, 0.0, 0.0])  # 예: [x, theta, x_dot, theta_dot]

    # 시뮬레이션 실행
    X, U, t_eval = cart_pole_nmpc.run_simulation(cart_pole_nmpc, x_init)

    # 시각화
    cart_pole_nmpc.visualize(X, U, t_eval)