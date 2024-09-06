import numpy as np

import casadi

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# linear inverted pendulum parameter
g = 9.81                # gravitational acceleration (m/s^2)
M = 1                   # cart mass (kg)
m = 0.2                 # pole mass (kg)
l = 1                   # pole length (m)
nu = 1                  # cantrol variable dimensions
nx = 4                  # state variable dimensions

# cost function weights
Q = casadi.diag([2.5, 10, 0.01, 0.01])
Q_f = casadi.diag([2.5, 10, 0.01, 0.01])
R = casadi.diag([0.1])

# prediction horizon etc.
T = 1
K = 20
dt = T / K

# constraints
x_lb = [-np.inf, -np.inf, -np.inf, -np.inf]
x_ub = [np.inf, np.inf, np.inf, np.inf]
u_lb = [-15]
u_ub = [15]

# target value
x_ref = casadi.DM([0, 0, 0, 0])
u_ref = casadi.DM([0])

total = (K + 1) * nx + K * nu # dimentsions of optimization variables


# state equation, integrator, and cost function
def make_f():
    states = casadi.SX.sym("states", nx)
    ctrls = casadi.SX.sym("ctrls", nu)

    x = states[0]
    theta = states[1]
    x_dot = states[2]
    theta_dot = states[3]
    F = ctrls[0]

    sin = casadi.sin(theta)
    cos = casadi.cos(theta)
    det = M + m * sin**2

    x_ddot = (m * l * sin * theta_dot**2 - m * g * sin * cos + F) / det
    theta_ddot = (- m * l * sin * cos * theta_dot**2 + (M + m) * g * sin - F * cos) / (l * det)

    states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)

    f = casadi.Function("f", [states, ctrls], [states_dot], ['x', 'u'], ['x_dot'])

    return f

def make_F_RK4():
    states = casadi.SX.sym("states", nx)
    ctrls = casadi.SX.sym("ctrls", nu)

    f = make_f()

    r1 = f(x = states, u = ctrls)["x_dot"]
    r2 = f(x = states + dt * r1 / 2, u = ctrls)["x_dot"]
    r3 = f(x = states + dt * r2 / 2, u = ctrls)["x_dot"]
    r4 = f(x = states + dt * r3, u = ctrls)["x_dot"]

    states_next = states + dt * (r1 + 2 * r2 + 2 * r3 + r4) / 6

    F_RK4 = casadi.Function("F_RK4", [states, ctrls], [states_next], ["x", "u"], ["x_next"])

    return F_RK4

def make_integrator():
    states = casadi.SX.sym("states", nx)
    ctrls = casadi.SX.sym("ctrls", nu)

    f = make_f()
    ode = f(x = states, u = ctrls)["x_dot"]

    dae = {"x" : states, "p" : ctrls, "ode" : ode}

    I = casadi.integrator("I", "cvodes", dae, 0, dt)

    return I

def compute_state_cost(x, u):
    x_diff = x - x_ref
    u_diff = u - u_ref
    cost = (casadi.dot(Q@x_diff, x_diff)) + casadi.dot(R@u_diff, u_diff) / 2

    return cost

def compute_terminal_cost(x):
    x_diff = x - x_ref
    cost = casadi.dot(Q_f@x_diff, x_diff) / 2

    return cost

# x1 = casadi.DM([1, casadi.pi / 6, 0.001, 0.01])
# u1 = casadi.DM([2])

# f = make_f()
# res = f(x = x1, u = u1)["x_dot"]
# print(res)
# F_RK4 = make_F_RK4()
# res1 = F_RK4(x = x1, u = u1)["x_next"]
# print(res1)
# I = make_integrator()
# xf = I(x0 = x1, p = u1)["xf"]
# print(xf)





# optimization problem
def make_nlp():
    F_RK4 = make_F_RK4()

    U = [casadi.SX.sym(f"u_{k}", nu) for k in range(K)]
    X = [casadi.SX.sym(f"x_{k}", nx) for k in range(K + 1)]
    G = []

    J = 0

    for k in range(K):
        J += compute_state_cost(X[k], U[k]) * dt
        eq = X[k + 1] - F_RK4(x = X[k], u = U[k])["x_next"]
        G.append(eq)
    
    J += compute_terminal_cost(X[-1])

    option = {'print_time' : False, 'ipopt' : {'max_iter' : 10, 'print_level' : 0}}
    nlp = {"x" : casadi.vertcat(*X, *U), "f" : J, "g" : casadi.vertcat(*G)}
    S = casadi.nlpsol("S", "ipopt", nlp, option)

    return S

# control input calculation
def comput_optimal_control(S, x_init, x0):
    x_init = x_init.full().ravel().tolist()

    lbx = x_init + K * x_lb + K * u_lb
    ubx = x_init + K * x_ub + K * u_ub
    lbg = [0] * K * nx
    ubg = [0] * K * nx

    res = S(lbx = lbx, ubx = ubx, lbg = lbg, ubg = ubg, x0 = x0)

    offset = (K + 1) * nx
    x0 = res["x"]
    u_opt = x0[offset : offset + nu]
    return u_opt, x0

# offset = (K + 1) * nx
# print(offset)


# MPC
S = make_nlp() 
t_span = [0, 10]
t_eval = np.arange(*t_span, dt)

x_init = casadi.DM([0, np.pi, 0, 0]) # initial value
x0 = casadi.DM.zeros(total)

I = make_integrator()

X = [x_init]
U = []
x_current = x_init
for t in t_eval:
    u_opt, x0 = comput_optimal_control(S, x_current, x0)
    x_current = I(x0 = x_current, p = u_opt)["xf"]
    X.append(x_current)
    U.append(u_opt)


tt = [1, 2, 3, 4, 5]
print(np.array(tt).size)
tt.pop()
print(np.array(tt).size)


# visualizing the results
X.pop()
X = np.array(X).reshape(t_eval.size, nx)
U = np.array(U).reshape(t_eval.size, nu)

plt.figure(figsize = (12, 4))
plt.subplot(1, 2, 1)

for k in range(nx):
    plt.plot(t_eval, X[:, k], label = f"x_{k}")
plt.legend()
plt.xlabel("Time")
plt.ylabel("state")

plt.subplot(1, 2, 2)
for k in range(nu):
    plt.step(t_eval, U[:, k], linestyle = "--", label = f"u_{k}")
plt.legend()
plt.xlabel("Time")
plt.ylabel("Control")

# plt.savefig("images/mpc.png")
plt.show()


# creating animations
fig = plt.figure(figsize = (12, 6))
ax = fig.add_subplot(111)
frames = np.arange(0, t_eval.size)
fps = 1 / dt

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
    ax.set_title(f"t{t_eval[i] : 0.2f}")

    x, theta, _, _ = X[i]
    u, = U[i]

    points = np.array([
        [x, x - l * np.sin(theta)],
        [0, l * np.cos(theta)]
    ])

    ax.hlines(0, x_lim_min, x_lim_max, colors = "black")
    ax.scatter(*points, color = "blue", s = 50)
    ax.plot(*points, color = "blue", lw = 2)
    ax.arrow(x, 0, u / u_scale, 0, width = 0.02, head_width = 0.06, head_length = 0.12) # length_includes_...

    w = 0.2
    h = 0.1
    rect = patches.Rectangle(xy = (x - w / 2, - h / 2), width = w, height = h, color = "black")
    ax.add_patch(rect)

ani = FuncAnimation(fig, update_figure, frames = frames)
ani.save("cart_pole.gif", writer = "pillow", fps = fps)

