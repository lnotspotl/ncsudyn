import casadi as ca
import matplotlib.pyplot as plt
import numpy as np

# Constants
nq = 3 + 4
nv = 3
nu = 2 + 2 + 4
g = 9.81

# Physical parameters
m = 12.0  # kg (similar to mini cheetah)
L = 0.6  # body length (m)
I = 1 / 12 * m * (L**2)  # moment of inertia for rectangular body

leg_length = 0.3  # m


# Getters
def com_position(q):
    return q[0], q[1]


# COM to leg
def leg_position(q, multiplier):
    if isinstance(multiplier, np.ndarray):
        lib = np
    else:
        lib = ca
    com_x, com_y = com_position(q)
    phi = q[2]
    theta1 = q[3]
    theta2 = q[4]

    if multiplier == -1:
        theta1 = q[5]
        theta2 = q[6]
    else:
        theta1 = q[3]
        theta2 = q[4]

    LL = multiplier * L

    shoulder_front_x = com_x + LL / 2 * lib.cos(phi)
    shoulder_front_y = com_y + LL / 2 * lib.sin(phi)

    knee_front_x = shoulder_front_x + leg_length * lib.cos(phi - lib.pi / 2 * 1 + theta1)
    knee_front_y = shoulder_front_y + leg_length * lib.sin(phi - lib.pi / 2 * 1 + theta1)

    ankle_front_x = knee_front_x + leg_length * lib.cos(phi - lib.pi / 2 * 1 + theta1 + theta2)
    ankle_front_y = knee_front_y + leg_length * lib.sin(phi - lib.pi / 2 * 1 + theta1 + theta2)

    return ankle_front_x, ankle_front_y


def leg_position_local(q, multiplier):
    com_x, com_y = com_position(q)
    return leg_position(q, multiplier) - np.array([com_x, com_y])


def rear_leg_position(q):
    return leg_position(q, -1)


def front_leg_position(q):
    return leg_position(q, 1)


def front_leg_position_local(q):
    return leg_position_local(q, 1)


def rear_leg_position_local(q):
    return leg_position_local(q, -1)


def euler_integrate(q, v, u, dt):
    x, y, phi = q[0], q[1], q[2]
    vx, vy, omega = v[0], v[1], v[2]
    theta1, theta2, theta3, theta4 = q[3], q[4], q[5], q[6]
    dtheta1, dtheta2, dtheta3, dtheta4 = u[4], u[5], u[6], u[7]
    F1_x, F1_y, F2_x, F2_y = u[0], u[1], u[2], u[3]

    # Integrate
    x_next = x + vx * dt
    y_next = y + vy * dt
    phi_next = phi + omega * dt

    theta1_next = theta1 + dtheta1 * dt
    theta2_next = theta2 + dtheta2 * dt
    theta3_next = theta3 + dtheta3 * dt
    theta4_next = theta4 + dtheta4 * dt

    ax = (F1_x + F2_x) / m
    ay = (F1_y + F2_y) / m - g  # gravity

    rfront_x, rfront_y = front_leg_position_local(q)
    rrear_x, rrear_y = rear_leg_position_local(q)

    wrench = (F1_x * rfront_y - F1_y * rfront_x) + (F2_x * rrear_y - F2_y * rrear_x)
    eps = 1 / I * wrench

    vx_next = vx + ax * dt
    vy_next = vy + ay * dt
    omega_next = omega + eps * dt

    q_next = np.array([x_next, y_next, phi_next, theta1_next, theta2_next, theta3_next, theta4_next])
    v_next = np.array([vx_next, vy_next, omega_next])

    return q_next, v_next


def draw_quadruped(q, u=None):
    com_x = q[0]
    com_y = q[1]
    phi = q[2]
    theta1 = q[3]
    theta2 = q[4]
    theta3 = q[5]
    theta4 = q[6]

    shoulder_front_x = com_x + L / 2 * np.cos(phi)
    shoulder_front_y = com_y + L / 2 * np.sin(phi)

    shoulder_rear_x = com_x - L / 2 * np.cos(phi)
    shoulder_rear_y = com_y - L / 2 * np.sin(phi)

    knee_front_x = shoulder_front_x + leg_length * np.cos(phi - np.pi / 2 * 1 + theta1)
    knee_front_y = shoulder_front_y + leg_length * np.sin(phi - np.pi / 2 * 1 + theta1)

    knee_rear_x = shoulder_rear_x + leg_length * np.cos(phi - np.pi / 2 * 1 + theta3)
    knee_rear_y = shoulder_rear_y + leg_length * np.sin(phi - np.pi / 2 * 1 + theta3)

    ankle_front_x = knee_front_x + leg_length * np.cos(phi - np.pi / 2 * 1 + theta1 + theta2)
    ankle_front_y = knee_front_y + leg_length * np.sin(phi - np.pi / 2 * 1 + theta1 + theta2)

    ankle_rear_x = knee_rear_x + leg_length * np.cos(phi - np.pi / 2 * 1 + theta3 + theta4)
    ankle_rear_y = knee_rear_y + leg_length * np.sin(phi - np.pi / 2 * 1 + theta3 + theta4)

    # Plot COM
    plt.plot(com_x, com_y, "ko", markersize=10)

    # Plot shoulders
    plt.plot(shoulder_front_x, shoulder_front_y, "bo", markersize=10)
    plt.plot(shoulder_rear_x, shoulder_rear_y, "bo", markersize=10)
    plt.plot([shoulder_front_x, shoulder_rear_x], [shoulder_front_y, shoulder_rear_y], "r-", linewidth=2)

    # Plot knees
    plt.plot(knee_front_x, knee_front_y, "ro", markersize=10)
    plt.plot([shoulder_front_x, knee_front_x], [shoulder_front_y, knee_front_y], "r-", linewidth=2)

    plt.plot(knee_rear_x, knee_rear_y, "ro", markersize=10)
    plt.plot([shoulder_rear_x, knee_rear_x], [shoulder_rear_y, knee_rear_y], "r-", linewidth=2)

    # Plot ankles
    plt.plot(ankle_front_x, ankle_front_y, "go", markersize=10)
    plt.plot([knee_front_x, ankle_front_x], [knee_front_y, ankle_front_y], "r-", linewidth=2)

    plt.plot(ankle_rear_x, ankle_rear_y, "go", markersize=10)
    plt.plot([knee_rear_x, ankle_rear_x], [knee_rear_y, ankle_rear_y], "r-", linewidth=2)

    com_x, com_y = com_position(q)

    if u is not None:
        F1_x, F1_y, F2_x, F2_y = u[:4] / 100
        plt.plot([ankle_front_x, ankle_front_x + F1_x], [ankle_front_y, ankle_front_y + F1_y], "r-", linewidth=2)
        plt.plot([ankle_rear_x, ankle_rear_x + F2_x], [ankle_rear_y, ankle_rear_y + F2_y], "r-", linewidth=2)

    # Draw ground
    plt.plot([-1, 0.2], [0.55, 0.55], "k-", linewidth=2)
    plt.plot([0.2, 0.2], [0.55, 0.75], "k-", linewidth=2)
    plt.plot([0.2, 2.0], [0.75, 0.75], "k-", linewidth=2)


N = 100
dt = 0.01
q = ca.SX.sym("q", nq)
v = ca.SX.sym("v", nv)
u = ca.SX.sym("u", nu)
q_next, v_next = euler_integrate(q, v, u, dt)
get_next_state = ca.Function("get_next_state", [q, v, u], [q_next, v_next])

opti = ca.Opti()

q_init = np.array([0.5, 1, 0, -1, 1.4, -1, 1.4])

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)
ax.set_xlim(-1, 2)
ax.set_ylim(-0.5, 2)
ax.set_aspect("equal")
draw_quadruped(q_init)
plt.show()

Q = opti.variable(nq, N)
V = opti.variable(nv, N)
U = opti.variable(nu, N - 1)

# Initial state
q_init = np.array([-0.1, 1, 0, -1, 1.4, -1, 1.4])

v_init = np.array([0, 0, 0])
opti.subject_to(Q[:, 0] == q_init)
opti.subject_to(V[:, 0] == v_init)

current_front_leg_x, current_front_leg_y = front_leg_position(q_init)
current_rear_leg_x, current_rear_leg_y = rear_leg_position(q_init)

q_init2 = q_init.copy()
opti.subject_to(Q[:, -1] == q_init2)
opti.subject_to(V[:, -1] == v_init)


for k in range(N - 1):
    q_next, v_next = get_next_state(Q[:, k], V[:, k], U[:, k])
    opti.subject_to(Q[:, k + 1] == q_next)
    opti.subject_to(V[:, k + 1] == v_next)

    # opti.subject_to(Q[1, k] >= 0.8)

    front_leg_x, front_leg_y = front_leg_position(Q[:, k])
    rear_leg_x, rear_leg_y = rear_leg_position(Q[:, k])

    mu = 0.9
    fx = U[0, k]
    fy = U[1, k]
    opti.subject_to(fx >= -mu * fy)
    opti.subject_to(fx <= mu * fy)

    fx = U[2, k]
    fy = U[3, k]

    opti.subject_to(fx >= -mu * fy)
    opti.subject_to(fx <= mu * fy)
    opti.subject_to(front_leg_x == current_front_leg_x)
    opti.subject_to(front_leg_y == current_front_leg_y)
    opti.subject_to(rear_leg_x == current_rear_leg_x)
    opti.subject_to(rear_leg_y == current_rear_leg_y)


cost = 0
q_init[1] = 0
for k in range(N - 1):
    position = Q[:2, k]
    desired_position = np.array([0.5, 0.5])
    cost += 100 * ca.sumsqr(position - desired_position)

    phi = Q[2, k]
    phi_desired = k / N * 2 * np.pi
    cost += 200 * (phi - phi_desired) ** 2
opti.minimize(cost)


initial_guess = np.zeros((nu, N - 1))
initial_guess[1, :] = m * 9.81 / 2
initial_guess[3, :] = m * 9.81 / 2

opti.set_initial(U, initial_guess)


opti.solver("ipopt")
sol = opti.solve()

U_sol = sol.value(U)
print(U_sol[:4, :])
print(U_sol[:4, :] >= -1e-4)

Q_sol = sol.value(Q)

# Create animation
import matplotlib.animation as animation

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)
ax.set_xlim(-1, 2)
ax.set_ylim(-0.5, 2)
ax.set_aspect("equal")


def animate(i):
    ax.clear()
    ax.set_xlim(-1, 2)
    ax.set_ylim(-0.5, 2)
    ax.set_aspect("equal")
    if i < N - 1:
        draw_quadruped(Q_sol[:, i], U_sol[:, i])
    else:
        draw_quadruped(Q_sol[:, i])
    return (ax,)


anim = animation.FuncAnimation(fig, animate, frames=N, interval=50, blit=False)
plt.show()
print(U_sol[:, -1])
