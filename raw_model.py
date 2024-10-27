import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from visualization import PendulumAnimator, visualize

# 定义动力学模型
def dynamics(t, state, u,):
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

# 定义初始状态 [x, x_dot, theta, theta_dot]
initial_state = [0, 0, np.pi/6, 0]  # 初始偏角为30度（可以修改）

# 时间范围
t_span = (0, 10)  # 仿真 10 秒
t_eval = np.linspace(t_span[0], t_span[1], 500)  # 生成时间点

# 控制输入 u (可以是常数，也可以根据需要定义为函数)
u = 0  # 恒定输入

# 进行仿真
sol = solve_ivp(dynamics, t_span, initial_state, args=(u,), t_eval=t_eval)

# 绘制结果
visualize(sol.y[0], sol.y[1], sol.y[2], sol.y[3], t_eval)
PendulumAnimator(sol.y[0], sol.y[2], t_eval, initial_state).start()