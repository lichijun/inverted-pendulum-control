import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# 定义系统矩阵 A 和 B
A = np.array([[0, 1, 0, 0],
              [0, 0, -0.363, 0],
              [0, 0, 0, 1],
              [0, 0, 15.244, 0]])

B = np.array([[0],
              [0.494],
              [0],
              [-0.741]])

# 定义动力学模型
def dynamics(t, state, u):
    X = np.array(state).reshape(4, 1)
    u = np.array([[u]])  # 控制输入 u
    dXdt = A @ X + B @ u
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
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(sol.t, sol.y[0, :], label='x (Position)')
plt.plot(sol.t, sol.y[2, :], label='theta (Angle)')
plt.xlabel('Time [s]')
plt.ylabel('Position / Angle')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(sol.t, sol.y[1, :], label='x_dot (Velocity)')
plt.plot(sol.t, sol.y[3, :], label='theta_dot (Angular Velocity)')
plt.xlabel('Time [s]')
plt.ylabel('Velocity / Angular Velocity')
plt.legend()

plt.tight_layout()
plt.show()