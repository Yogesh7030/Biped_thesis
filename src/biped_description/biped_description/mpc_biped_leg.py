#!/usr/bin/env python3

import casadi as ca
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time as tm

class LiveJointPublisher(Node):
    def __init__(self):
        super().__init__('live_joint_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10
        )

    def publish_joint_angles(self, hip, knee, ankle):
        msg = Float64MultiArray()
        msg.data = [hip, knee, ankle]
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    publisher_node = LiveJointPublisher()

    g = 9.81 # gravity
    z_c = 0.8 # COM height
    omega = np.sqrt(g / z_c)
    dt = 0.15
    N = 30 # prediction horizon
    mass = 60  # mass of the biped (kg)

    # Continuous footstep generation
    step_interval = 12
    stride = 0.15
    sim_steps = 60  # Increased for longer walking

    # initial states
    x0 = np.array([0.0,0.1]) # x, dx
    y0 = np.array([0.0,0.1]) # y, dy

    # MPC setup
    def setup_mpc(n_x, n_u, A, B, Q, R, Q_terminal):
        X = ca.SX.sym('X', n_x, N + 1)
        U = ca.SX.sym('U', n_u, N)
        x0_param = ca.SX.sym('x0_param', n_x)
        ref_param = ca.SX.sym('ref_param', n_x)

        cost = 0
        g = [X[:, 0] - x0_param]
        for k in range(N):
            xk = X[:, k]
            uk = U[:, k]
            cost += ca.mtimes([(xk - ref_param).T, Q, (xk - ref_param)]) + ca.mtimes([uk.T, R, uk])
            x_next = ca.mtimes(A, xk) + ca.mtimes(B, uk)
            g.append(X[:, k + 1] - x_next)
        cost += ca.mtimes([(X[:, N] - ref_param).T, Q_terminal, (X[:, N] - ref_param)])

        vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        params = ca.vertcat(x0_param, ref_param)
        nlp = {'x': vars, 'p': params, 'f': cost, 'g': ca.vertcat(*g)}
        solver = ca.nlpsol('solver', 'ipopt', nlp, {'ipopt.print_level': 0, 'print_time': 0})
        return solver

    A_x = np.array([[1, dt], [omega**2 * dt, 1]])
    B_x = np.array([[0], [-omega**2 * dt]])
    Q_x = np.diag([200, 100])
    R_x = np.diag([10])
    Qx_term = np.diag([150, 60])
    solver_x = setup_mpc(2, 1, A_x, B_x, Q_x, R_x, Qx_term)

    A_y = A_x.copy()
    B_y = B_x.copy()
    Q_y = np.diag([100, 20])
    R_y = np.diag([100])
    Qy_term = np.diag([400, 200])
    solver_y = setup_mpc(2, 1, A_y, B_y, Q_y, R_y, Qy_term)

    x_sym = ca.SX.sym('x', 2)
    u_sym = ca.SX.sym('u', 1)
    f_x = ca.Function('f_x', [x_sym, u_sym], [ca.mtimes(A_x, x_sym) + ca.mtimes(B_x, u_sym)])

    y_sym = ca.SX.sym('y', 2)
    f_y = ca.Function('f_y', [y_sym, u_sym], [ca.mtimes(A_y, y_sym) + ca.mtimes(B_y, u_sym)])

    def plan_next_foot(foot_idx, side, prev_foot):
        """Plan next footstep (swing foot only)"""
        px = prev_foot[0] + stride
        py = 0.1 * side  # alternate L/R
        return np.array([px, py])

    def shift_guess(prev_sol, n_x, n_u, N):
        X_flat = prev_sol[:n_x*(N+1)].reshape((n_x, N+1))
        U_flat = prev_sol[n_x*(N+1):].reshape((n_u, N))
        X_new = np.hstack((X_flat[:, 1:], X_flat[:, -1:]))
        U_new = np.hstack((U_flat[:, 1:], U_flat[:, -1:]))
        return np.vstack((X_new.reshape(-1,1), U_new.reshape(-1,1)))

    x_current = x0.copy()
    y_current = y0.copy()
    X_hist = []
    Y_hist = []
    U_px = []
    U_py = []
    state_full = []

    # initial foot plan
    foot_idx = 0
    side     = 1
    prev_foot = np.array([0.0, 0.0])  # starting foot location
    P_plan = [prev_foot.copy()]

    zmp_margin = 0.25

    def init_guess(n_u, x_init):
        X_init = np.tile(x_init.reshape(-1, 1), (1, N + 1))
        U_init = np.zeros((n_u, N))
        return np.vstack((X_init.reshape(-1, 1), U_init.reshape(-1, 1)))

    vars_init_x = init_guess(1, x_current)
    vars_init_y = init_guess(1, y_current)

    for step in range(sim_steps):
        # replan every step_interval
        if step % step_interval == 0 and step > 0:
            foot_idx += 1
            side *= -1
            new_foot = plan_next_foot(foot_idx, side, P_plan[-1])
            P_plan.append(new_foot)

        # current support foot from plan
        foot_px, foot_py = P_plan[-1]
        
        external_force = np.array([0.0, 0.0])
        if step == 30:
            external_force[0] = 20.0
        if step == 50:
            external_force[1] = 10.0

        x_ref = np.array([foot_px, 0.0])
        y_ref = np.array([foot_py, 0.0])
        lb_x = [-ca.inf, -1.0] * (N + 1) + [foot_px - zmp_margin] * N
        ub_x = [ca.inf, 1.0] * (N + 1) + [foot_px + zmp_margin] * N
        lb_y = [-ca.inf, -0.6] * (N + 1) + [foot_py - zmp_margin] * N
        ub_y = [ca.inf, 0.6] * (N + 1) + [foot_py + zmp_margin] * N

        p_x = np.concatenate((x_current, x_ref))
        sol_x = solver_x(x0=vars_init_x, p=p_x, lbg=[0]*((2)*(N+1)), ubg=[0]*((2)*(N+1)), lbx=lb_x, ubx=ub_x)
        sol_vec_x = np.array(sol_x['x'].full()).flatten()
        U_sol_x = sol_vec_x[2*(N+1):].reshape((1, N))
        u_px = np.clip(U_sol_x[0, 0], foot_px - zmp_margin, foot_px + zmp_margin)

        p_y = np.concatenate((y_current, y_ref))
        sol_y = solver_y(x0=vars_init_y, p=p_y, lbg=[0]*((2)*(N+1)), ubg=[0]*((2)*(N+1)), lbx=lb_y, ubx=ub_y)
        sol_vec_y = np.array(sol_y['x'].full()).flatten()
        U_sol_y = sol_vec_y[2*(N+1):].reshape((1, N))
        u_py = np.clip(U_sol_y[0, 0], foot_py - zmp_margin, foot_py + zmp_margin)

        x_current[1] += (external_force[0] / mass) * dt
        y_current[1] += (external_force[1] / mass) * dt

        x_current = np.array(f_x(x_current, u_px)).flatten()
        y_current = np.array(f_y(y_current, u_py)).flatten()

        X_hist.append(x_current)
        Y_hist.append(y_current)
        U_px.append(u_px)
        U_py.append(u_py)
        state_full.append(np.concatenate((x_current, y_current)))

        prev_sol_x = sol_vec_x
        prev_sol_y = sol_vec_y
        vars_init_x = shift_guess(prev_sol_x, 2, 1, N)
        vars_init_y = shift_guess(prev_sol_y, 2, 1, N)

    X_hist = np.array(X_hist).T
    Y_hist = np.array(Y_hist).T
    U_px = np.array(U_px)
    U_py = np.array(U_py)
    state_full = np.array(state_full).T
    time_history = np.arange(X_hist.shape[1]) * dt

    df = pd.DataFrame({
        'time': time_history,
        'x': X_hist[0], 'dx': X_hist[1],
        'y': Y_hist[0], 'dy': Y_hist[1],
        'px': U_px, 'py': U_py
    })

    P_arr = np.array(P_plan).T  # 2xM

    # Compute swing foot trajectory (simplified linear swing)
    swing_traj_x = []
    swing_traj_y = []
    swing_traj_z = []
    swing_times = []

    for i in range(1, P_arr.shape[1]):
        # time span for each step
        t_start = i * step_interval * dt
        t_end = (i + 1) * step_interval * dt
        t = np.linspace(t_start, t_end, step_interval)

        prev_foot = P_arr[:, i - 1]
        next_foot = P_arr[:, i]

        # linear interpolation
        x_traj = np.linspace(prev_foot[0], next_foot[0], step_interval)
        y_traj = np.linspace(prev_foot[1], next_foot[1], step_interval)
        
        # Sinusoidal arc for Z (0 to max height and back down)
        z_traj = 0.07 * np.sin(np.pi * np.linspace(0, 1, step_interval))  # ~5 cm swing

        swing_traj_x.extend(x_traj)
        swing_traj_y.extend(y_traj)
        swing_traj_z.extend(z_traj)
        swing_times.extend(t)

    swing_traj_x = np.array(swing_traj_x)
    swing_traj_y = np.array(swing_traj_y)
    swing_traj_z = np.array(swing_traj_z)
    swing_times = np.array(swing_times)

    # Joint limits from your URDF (radians)
    hip_min,   hip_max   = -1.57,  1.57   # left_hip_roll_joint
    knee_min,  knee_max  = -1.57,  0.00   # left_knee_joint
    ank_min,   ank_max   = -0.50,  0.50   # left_ankle_joint

    def inverse_kinematics_3link_planar(px, pz, phi, l1, l2, l3):
        """
        Compute joint angles for a 3-link planar leg (hip, knee, ankle) to reach foot pose (px, pz, phi).
        l1 = thigh, l2 = shin, l3 = foot.
        Returns: (theta1, theta2, theta3), already clamped to physical limits.
        """
        # 1) Compute the "wrist" position (ankle joint center)
        wx = px - l3 * np.cos(phi)
        wz = pz - l3 * np.sin(phi)
        r  = np.hypot(wx, wz)

        # 2) Enforce two-link reach [|l1-l2|, l1+l2]
        r_max = l1 + l2
        r_min = abs(l1 - l2)
        if r > r_max:
            wx *= (r_max / r);  wz *= (r_max / r)
        elif r < r_min:
            wx *= (r_min / r);  wz *= (r_min / r)

        # 3) Law‐of‐cosines, with D clamped
        D = (wx**2 + wz**2 - l1**2 - l2**2) / (2 * l1 * l2)
        D = np.clip(D, -1.0, 1.0)

        theta2 = np.arctan2(-np.sqrt(max(0, 1 - D**2)), D)
        k1     = l1 + l2 * np.cos(theta2)
        k2     = l2 * np.sin(theta2)
        theta1 = np.arctan2(wz, wx) - np.arctan2(k2, k1)
        theta3 = phi - theta1 - theta2

        # 4) Clamp to your URDF‐defined joint limits
        theta1 = np.clip(theta1, hip_min,   hip_max)
        theta2 = np.clip(theta2, knee_min,  knee_max)
        theta3 = np.clip(theta3, ank_min,   ank_max)

        return theta1, theta2, theta3

    hip_traj = []
    knee_traj  = []
    ankle_traj = []
    for idx in range(len(swing_traj_x)):
        theta1, theta2, theta3 = inverse_kinematics_3link_planar(
            px  = swing_traj_x[idx],
            pz  = swing_traj_z[idx],
            phi = 0.0,
            l1  = 0.22,
            l2  = 0.28,
            l3  = 0.05
        )
        publisher_node.publish_joint_angles(theta1, theta2, theta3)
        rclpy.spin_once(publisher_node, timeout_sec=0.0)
        tm.sleep(dt)  # Keep pace with simulation

        hip_traj.append(theta1)
        knee_traj.append(theta2)
        ankle_traj.append(theta3)

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()