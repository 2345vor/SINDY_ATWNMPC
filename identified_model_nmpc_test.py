import casadi as ca
import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt
import json
import os
import argparse
import sys

# 下面的模型和轨迹参数方程是以厘米为单位，时间以秒为单位

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='NMPC轨迹跟踪控制验证')
    parser.add_argument('--model', type=int, choices=[1, 2, 3], default=1,
                       help='模型类别 (1: 基础模型18参数, 2: 分离模型21参数, 3: 简化模型16参数)')
    parser.add_argument('--trajectory', type=int, choices=[1, 2, 3], default=3,
                       help='跟踪曲线 (1: 椭圆, 2: 正弦直线, 3: 双正弦)')
    parser.add_argument('--predict_step', type=int, default=10,
                       help='预测步长,最大值20,最小值5,默认10')
    parser.add_argument('--dt', type=float, default=0.1,
                       help='采样时间,最大值0.5,最小值0.05,默认0.1')
    parser.add_argument('--cycle_time', type=int, default=45,
                       help='轨迹周期时间,最大值90,最小值45,默认45,推荐是3的倍数')
    parser.add_argument('--loop_num', type=int, default=1,
                       help='循环次数,最大值5,最小值1,默认1')
    parser.add_argument('--noise_mean', type=float, default=-2,
                       help='轨迹噪声均值,默认-2')
    parser.add_argument('--noise_std', type=float, default=0.52,
                       help='轨迹噪声标准差,默认0.5')
    parser.add_argument('--eta', type=float, default=100.0,
                   help='自适应NMPC控制参数,默认100.0（参考原文件的距离缩放系数）')
    parser.add_argument('--adaptive', action='store_true', default=True,
                       help='是否启用自适应NMPC控制')
    parser.add_argument('--output_dir', type=str, default='nmpc_results',
                       help='输出目录')
    return parser.parse_args()

def get_trajectory_equations(trajectory_type, t):
    """根据轨迹类型生成轨迹方程"""
    if trajectory_type == 1:
        # 椭圆轨迹
        x_r = 150 * np.sin(t) + 28.2
        y_r = 60 * np.cos(t) - 85.4
        name = "Elliptical Trajectory: x = 150*sin(t) + 28.2, y = 60*cos(t) - 85.4"
    elif trajectory_type == 2:
        # 正弦直线轨迹
        x_r = 40 * np.sin(t) + 1
        y_r = 10*t
        name = "SIN Trajectory: x = 40*sin(t) + 1, y = t"
    elif trajectory_type == 3:
        # 双正弦轨迹
        x_r = 150 * np.cos(t) + 28.2
        y_r = 60 * np.sin(2*t) - 85.4
        name = "Lissajous Trajectory: x = 150*sin(t) + 28.2, y = 60*cos(2*t) - 85.4"
    else:
        raise ValueError(f"不支持的轨迹类型: {trajectory_type}")
    
    return x_r, y_r, name

# 解析命令行参数
args = parse_arguments()

# 设置并创建输出目录
output_dir = args.output_dir
os.makedirs(output_dir, exist_ok=True)

# ==================== 常量定义 ====================
M_PI = np.pi
args.predict_step = min(max(args.predict_step, 5), 20)
args.dt = min(max(args.dt, 0.05), 0.5)
args.cycle_time = min(max(args.cycle_time, 45), 90)
args.loop_num = min(max(args.loop_num, 1), 5)
N = args.predict_step           # 预测步长
T = args.dt                     # 采样时间
motor_power = 500
eta = args.eta
Adaptive_flag = args.adaptive  # 使用命令行参数控制自适应功能
position_noise_mean = 0  # 移除系统性偏差，使用零均值噪声
position_noise_std = args.noise_std      # 位置噪声标准差
loop_num = args.loop_num
tol = args.cycle_time*loop_num  # 仿真总时间s
t = np.linspace(0, 2*M_PI*loop_num, int(tol / T) + 1)  # 时间向量

# ==================== 生成参考轨迹 ====================
# 使用指定的轨迹参数
x_r1, y_r1, trajectory_name = get_trajectory_equations(args.trajectory, t)

x_r1 = x_r1[:, np.newaxis]
y_r1 = y_r1[:, np.newaxis]
path_points = np.concatenate([x_r1, y_r1], axis=1)
sim_steps = len(path_points)

print(f"轨迹点数量: {sim_steps}")
print(f"轨迹参数: {trajectory_name}")
print(f"自适应控制: {'启用' if Adaptive_flag else '禁用'}")
print(f"模型类型: Model {args.model}")

# ==================== 权重参数 ====================
# 调整权重参数以匹配Simulation_nmpc0621.py的效果
if args.trajectory == 2 or args.trajectory == 1:
    Q = [0.001, 0.001, 0.001, 3, 3, 0.1]  # 降低位置误差权重
    R = [0.003, 0.003]  # 调整控制权重
    F_low = [0.001, 0.002, 0.002, 10, 10, 0.15]  # 参考原文件的Q_low
    F_high = [0.001, 0.002, 0.002, 5000, 5000, 0.15]  # 参考原文件的Q_high
elif args.trajectory == 3:
    Q = [0.001, 0.001, 0.001, 5, 5, 0.15]  # 稍微提高复杂轨迹的位置权重
    R = [0.003, 0.003]  # 统一控制权重
    F_low = [0.001, 0.002, 0.002, 10, 10, 0.15]
    F_high = [0.001, 0.002, 0.002, 1000, 1000, 0.15]

# ==================== 状态和控制变量定义 ====================
u = ca.SX.sym('u')
v = ca.SX.sym('v')
r = ca.SX.sym('r')
x_pos = ca.SX.sym('x')
y_pos = ca.SX.sym('y')
psi = ca.SX.sym('psi')
Tp = ca.SX.sym('Tp')
Ts = ca.SX.sym('Ts')

state = ca.vertcat(u, v, r, x_pos, y_pos, psi)
control = ca.vertcat(Tp, Ts)

# ==================== 使用识别的动力学模型 ====================
def build_dynamics_model(model, u, v, r, psi, Tp, Ts):
    """根据模型类型构建动力学模型"""
    
    if model == 1:
        #1模型
        du_model = -0.304252 * u - 0.999143 * v + 0.133762 * r * u + 3.182423 * (Tp + Ts) + 5.932517
        dv_model = -3.908326 * r - 0.343078 * v + 0.119421 * (Tp - Ts) + 0.036246
        dr_model = -0.000401 * u * v - 4.544756 * r - 0.000019 * (Tp - Ts) + 0.010658
        
        print(f"\n使用识别的模型参数 (Model 1 - SGF参数):")
                
    elif model == 2:
        # 2模型 
        du_model = 0.247237 * u * r + -0.241717 * u + -0.342290 * v + 0.006176 * (Tp + Ts) + 2.766341
        dv_model = -4.839525 * r + -0.554457 * v + -0.000211 * (Tp - Ts) + -0.337847
        dr_model = 0.004524 * u * v + -0.663275 * r + -0.000285 * (Tp - Ts) + -0.025273
        
        print(f"\n使用识别的模型参数 (Model 2 - EKF参数):")
                
    elif model == 3:
        # 3模型 
        du_model = 3.421511 * v * r + -0.387560 * u + 0.221420 * v + 6.025003 * r + 0.002711 * (Tp + Ts) + 7.635497
        dv_model = -0.279953 * u * r + -0.017586 * u + -0.963812 * v + -2.291902 * r + -0.000274 * (Tp - Ts) + 0.037983
        dr_model = 0.001568 * u * v + 0.005271 * u + 0.044927 * v + -0.442848 * r + -0.000233 * (Tp - Ts) + -0.155814
        
        print(f"\n使用识别的模型参数 (Model 3 - LPF参数):")
            
    else:
        raise ValueError(f"不支持的模型类型: {args.model}")
    
    # 构建完整的动力学模型
    rhs = ca.vertcat(
        du_model,
        dv_model,
        dr_model,
        # 位置和航向角的运动学方程
        u * ca.cos(psi) - v * ca.sin(psi),
        u * ca.sin(psi) + v * ca.cos(psi),
        r
    )
    
    return rhs

# 构建识别的动力学模型
rhs = build_dynamics_model(args.model, u, v, r, psi, Tp, Ts)

f = ca.Function('f', [state, control], [rhs])

# ==================== 初始状态 ====================
x0 = [0, 0, 0, path_points[0][0]+2, path_points[0][1]-2, 0]
xs_list = [[0, 0, 0, point[0], point[1], 0] for point in path_points]
state_history = [x0]
u0 = np.zeros(2 * N).tolist()
control_history = []

print(f"\n初始状态: {x0}")
print(f"目标轨迹起点: [{path_points[0][0]:.2f}, {path_points[0][1]:.2f}]")

# ==================== 构建NLP问题 ====================
U = ca.SX.sym('U', 2, N)
P_state = ca.SX.sym('P_state', 12)
P_weight = ca.SX.sym('P_weight', 14)
P = ca.vertcat(P_state, P_weight)

X = ca.SX.sym('X', 6, N + 1)
X[:, 0] = P_state[0:6]

for k in range(N):
    X[:, k + 1] = X[:, k] + T * f(X[:, k], U[:, k])

obj = 0
Qk = ca.diagcat(P_weight[0], P_weight[1], P_weight[2], P_weight[3], P_weight[4], P_weight[5])
Rk = ca.diagcat(P_weight[6], P_weight[7])
Qf = ca.diagcat(P_weight[8], P_weight[9], P_weight[10], P_weight[11], P_weight[12], P_weight[13])

for k in range(N):
    st_err = X[:, k] - P_state[6:12]
    con = U[:, k]
    obj += st_err.T @ Qk @ st_err + con.T @ Rk @ con

obj += (X[:, N] - P_state[6:12]).T @ Qf @ (X[:, N] - P_state[6:12])

g = []
for k in range(N + 1):
    # 添加五个状态约束：u, v, r, x, y
    g.append(X[0, k])  # u (surge velocity)
    g.append(X[1, k])  # v (sway velocity)
    g.append(X[2, k])  # r (yaw rate)
    g.append(X[3, k])  # x (position)
    g.append(X[4, k])  # y (position)
g = ca.vertcat(*g)

OPT_variables = ca.reshape(U, 2 * N, 1)
nlp = {'x': OPT_variables, 'f': obj, 'g': g, 'p': P}

opts = {
    'ipopt.print_level': 0,
    'print_time': 0,
    'ipopt.max_iter': 30,
    'ipopt.acceptable_tol': 1e-6
}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# ==================== 约束边界 ====================
lbx = [-motor_power,-motor_power] * (N)
ubx = [motor_power,motor_power] * (N)
lbg = [-250, -50, -0.3*M_PI,-210,-280] * (N+1)
ubg = [250, 50, 0.3*M_PI,210,0] * (N+1)

# ==================== 初始化误差记录 ====================
lateral_errors = []
heading_errors = []

def normalize_angle_diff(delta):
    if delta > M_PI:
        return delta - 2 * M_PI
    elif delta < -M_PI:
        return delta + 2 * M_PI
    else:
        return delta

def compute_target_psi(xs_list, sim_index):
    if sim_index >= len(xs_list) - 1:
        return xs_list[sim_index][5]
    current_x = xs_list[sim_index][3]
    current_y = xs_list[sim_index][4]
    next_x = xs_list[sim_index + 1][3]
    next_y = xs_list[sim_index + 1][4]
    return math.atan2(next_y - current_y, next_x - current_x)

print("\n开始NMPC仿真...")

# ==================== NMPC 仿真 ====================
for sim in range(sim_steps):
    if sim % 500 == 0:
        print(f"仿真进度: {sim}/{sim_steps} ({sim/sim_steps*100:.1f}%)")
    
    xs = xs_list[sim] if sim < len(xs_list) else xs_list[-1]

    current_x, current_y = x0[3], x0[4]
    target_x, target_y = xs[3], xs[4]
    x0[5] = normalize_angle_diff(x0[5])
    current_psi = x0[5]
    target_psi = compute_target_psi(xs_list, sim)
    distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
    lateral_errors.append(distance_error)

    delta_psi = normalize_angle_diff(target_psi-current_psi)
    heading_errors.append(abs(delta_psi))
    distance_coeff = max(0, min(1.0, distance_error / eta))
    if Adaptive_flag:
        Q_current = Q
        R_current = R
        F_current = [F_low[i] + distance_coeff * (F_high[i] - F_low[i]) for i in range(6)]
    else:
        Q_current = Q
        R_current = R
        F_current = [(F_high[i] + F_low[i])/2 for i in range(6)]
    p_state = x0 + xs
    p_weight = Q_current + R_current + F_current
    p = p_state + p_weight

    res = solver(x0=u0, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

    u_opt = res['x'].full().flatten().tolist()
    u0_step = u_opt[:2]
    # 直接使用优化结果，减少平滑导致的响应滞后
    u0_step = u_opt[:2]
    
    # 可选：轻微平滑（如果需要减少控制突变）
    # if len(control_history) > 0:
    #     prev_control = np.array(control_history[-1])
    #     current_control = np.array(u0_step)
    #     u0_step = (0.95 * current_control + 0.05 * prev_control).tolist()

    next_state = f(x0, u0_step)
    x_next = x0 + T * next_state.full().flatten()

    # 加入白噪声
    noise = np.random.normal(position_noise_mean, position_noise_std, size=2)
    x_next[3] += noise[0]
    x_next[4] += noise[1]
    x0 = x_next.tolist()
    control_history.append(u0_step)
    state_history.append(x0)
    u0 = u_opt[2:] + u_opt[-2:]

print("仿真完成！")

# ==================== 提取轨迹用于绘图 ====================
traj_u =   [s[0] for s in state_history][1:]
traj_v =   [s[1] for s in state_history][1:]
traj_r =   [s[2] for s in state_history][1:]
traj_x =   [s[3] for s in state_history][1:]
traj_y =   [s[4] for s in state_history][1:]
traj_psi = [s[5] for s in state_history][1:]
ref_x = [p[0] for p in path_points]
ref_y = [p[1] for p in path_points]
PWML = [u[0] for u in control_history]
PWMR = [u[1] for u in control_history]

# ==================== 输出统计数据 ====================
print(f"\n=== 跟踪性能统计 ===")
print(f"横向误差: 平均={np.mean(lateral_errors):.4f}, 标准差={np.std(lateral_errors):.4f}")
print(f"航向误差: 平均={np.mean(heading_errors):.4f} rad, 标准差={np.std(heading_errors):.4f} rad")
print(f"最大横向误差: {np.max(lateral_errors):.4f}")
print(f"最大航向误差: {np.max(heading_errors):.4f} rad")

# ==================== 绘图 ====================
plt.figure(figsize=(10, 6))
plt.plot(ref_y, ref_x, 'r--', linewidth=2, label='Reference Path')
plt.plot(traj_y, traj_x, 'b-', linewidth=1.5, label='NMPC Trajectory (Identified Model)')
plt.scatter([traj_y[0]], [traj_x[0]], c='g', s=100, label='Start', zorder=5)
plt.scatter([traj_y[-1]], [traj_x[-1]], c='orange', s=100, label='End', zorder=5)
plt.xlabel('East Position (m)')
plt.ylabel('North Position (m)')
plt.title(f'NMPC Path Tracking with Identified Model (NED Coordinate)\n{trajectory_name}')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')

# 保存轨迹对比图
trajectory_plot_path = os.path.join(output_dir, f"nmpc_trajectory_{args.model}_for_trajectory_{args.trajectory}.png")
plt.savefig(trajectory_plot_path)

# ==================== 误差绘图 ====================
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(lateral_errors, label='Lateral Tracking Error', color='b', linewidth=1)
plt.axhline(y=np.mean(lateral_errors), color='b', linestyle='--', 
           label=f'Mean Lateral Error: {np.mean(lateral_errors):.4f}')
plt.ylabel('Lateral Error')
plt.title('Tracking Errors with Identified Model')
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(2, 1, 2)
plt.plot(heading_errors, label='Heading Angle Error (rad)', color='r', linewidth=1)
plt.axhline(y=np.mean(heading_errors), color='r', linestyle='--', 
           label=f'Mean Heading Error: {np.mean(heading_errors):.4f} rad')
plt.xlabel('Time Step')
plt.ylabel('Heading Error (rad)')
plt.legend()
plt.grid(True, alpha=0.3)

# 保存误差图
error_plot_path = os.path.join(output_dir, f"nmpc_error_{args.model}_for_trajectory_{args.trajectory}.png")
plt.savefig(error_plot_path)

# 可视化状态变量
plt.figure(figsize=(12, 8))
plt.subplot(4, 1, 1)
plt.plot(t[:len(traj_u)], traj_u, label='u (surge velocity)', color='blue')
plt.ylabel('u (m/s)')
plt.title('State Variables with Identified Model')
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(4, 1, 2)
plt.plot(t[:len(traj_v)], traj_v, label='v (sway velocity)', color='green')
plt.ylabel('v (m/s)')
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(4, 1, 3)
plt.plot(t[:len(traj_r)], traj_r, label='r (yaw rate)', color='red')
plt.ylabel('r (rad/s)')
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(4, 1, 4)
plt.plot(t[:len(traj_psi)], traj_psi, label='psi (heading angle)', color='purple')
plt.ylabel('psi (rad)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

# 保存状态变量图
state_plot_path = os.path.join(output_dir, f"nmpc_state_variables_{args.model}_for_trajectory_{args.trajectory}.png")
plt.savefig(state_plot_path)

# 推进器输出对比
plt.figure(figsize=(12, 6))
t_plot = t[:len(PWML)]
plt.subplot(2, 1, 1)
plt.plot(t_plot, PWML, label='Ts (Starboard Thruster)', color='blue')
plt.ylabel('Thrust (N)')
plt.title('Thruster Outputs with Identified Model')
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(2, 1, 2)
plt.plot(t_plot, PWMR, label='Tp (Port Thruster)', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Thrust (N)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

# 保存推进器输出图
thruster_plot_path = os.path.join(output_dir, f"nmpc_thruster_outputs_{args.model}_for_trajectory_{args.trajectory}.png")
plt.savefig(thruster_plot_path)

plt.show()

# ==================== 保存结果 ====================
# 确保所有数组长度一致
min_length = min(len(traj_x), len(traj_y), len(traj_u), len(traj_v), len(traj_r), len(traj_psi), 
                len(PWMR), len(PWML), len(lateral_errors), len(heading_errors))

output_data = {
    'Time': t[:min_length],
    'X': traj_x[:min_length],
    'Y': traj_y[:min_length],
    'Ref_X': ref_x[:min_length],
    'Ref_Y': ref_y[:min_length],
    'u': traj_u[:min_length],
    'v': traj_v[:min_length],
    'r': traj_r[:min_length],
    'psi': traj_psi[:min_length],
    'Tp': PWMR[:min_length],
    'Ts': PWML[:min_length],
    'Lateral_Error': lateral_errors[:min_length],
    'Heading_Error': heading_errors[:min_length]
}

df_output = pd.DataFrame(output_data)

output_path = os.path.join(output_dir, f"nmpc_identified_model_{args.model}_for_trajectory_{args.trajectory}_results.csv")
df_output.to_csv(output_path, index=False)


print(f"\n使用识别模型的NMPC轨迹跟踪测试完成！")