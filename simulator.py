import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from visualization import PendulumAnimator, visualize
from controller_collection import OpenLoopController, PolePlacementController, LQRController,\
                                  PidController1, PidController2, PidController3, MPCController, PolePlacementControllerWithObserver

# 仿真模型
def dynamics(t, state, u):

    # 模型参数
    M = 2
    m = 0.1
    l = 0.5
    g = 9.8

    theta = state[2]
    theta_dot = state[3]
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    A0 = np.array([[1, 0, 0, 0],
                   [0, M + m, 0, m * l * cos_theta],
                   [0, 0, 1, 0],
                   [0, m * l * cos_theta, 0, (4 / 3) * m * l**2]])
    B0 = np.array([[0, -1, 0, 0],
                   [0, 0, 0, -m * l * theta_dot * sin_theta],
                   [0, 0, 0, -1],
                   [0, 0, 0, 0]])
    A = np.linalg.inv(A0) @ (-B0)
    Bu = np.linalg.inv(A0) @ np.array([0, u, 0, m * g * l * sin_theta]).reshape(4, 1)
    X = np.array(state).reshape(4, 1)

    dXdt = A @ X + Bu
    return dXdt.flatten()

# 仿真参数
initial_state = [0, 0, 5*np.pi/180, 0]  # 定义初始状态 [x, x_dot, theta, theta_dot]
sim_time = 30  # 仿真时间
sim_interval = 0.01 # 仿真时间间隔
control_interval = 0.01 # 控制时间间隔

# 控制器
# # 不加控制
# controller = OpenLoopController(0)

# # 极点配置控制
# desired_poles = np.array([-1, -2, complex(-1, np.sqrt(3)), complex(-1, -np.sqrt(3))])  
# controller = PolePlacementController(desired_poles)

# # LQR控制
# matrix_Q = np.array([[2, 0, 0, 0],
#                                 [0, 0.1, 0, 0],
#                                 [0, 0, 10, 0],
#                                 [0, 0, 0, 0.2]])
            
# matrix_R = np.array([[1]])
# controller = LQRController(matrix_Q, matrix_R)

# # PID控制(角度环)
# controller = PidController1(25, 0, 100, 0, control_interval)

# # PID控制(串级PID,角度环+角速度环)
# controller = PidController2(5, 0, 20, 0, 10, 0, 20, 0, control_interval)

# # PID控制(串级PID,位置环+角度环+角速度环)
# controller = PidController3(0.01, 0, 5, 0, 5, 0, 20, 0, 10, 0, 20, 0, control_interval)

# # MPC控制
# controller = MPCController(sim_interval)

# 极点配置控制（带状态观测器） 
controller = PolePlacementControllerWithObserver(sim_interval, initial_state)

# 开始仿真
state = initial_state
sim_result = [state]
sim_t = np.arange(0, sim_time, sim_interval)
sim_t = sim_t.tolist()
sim_t.append(sim_time)

for t in np.arange(0, sim_time, sim_interval):
    u = controller.control(t, state)
    sol = solve_ivp(dynamics, (t, t+sim_interval), state, args=(u,))
    state = [row[-1] for row in sol.y]
    sim_result.append(state)

# 仿真结果
sim_disp = [row[0] for row in sim_result]
sim_disp_dot = [row[1] for row in sim_result]
sim_theta = [row[2] for row in sim_result]
sim_theta_dot = [row[3] for row in sim_result]
visualize(sim_disp, sim_disp_dot, sim_theta, sim_theta_dot, sim_t)
PendulumAnimator(sim_disp, sim_theta, sim_t, initial_state).start()