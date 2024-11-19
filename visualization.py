import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from typing import List

def visualize(displacements: List[float], velocity: List[float], angle: List[float], angler_velocity: List[float], t_eval: List[float]):
    plt.figure(figsize=(10, 8))
    plt.subplot(2, 1, 1)
    plt.plot(t_eval, displacements, label='x (Position)')
    plt.plot(t_eval, angle, label='theta (Angle)')
    plt.xlabel('Time [s]')
    plt.ylabel('Position / Angle')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(t_eval, velocity, label='x_dot (Velocity)')
    plt.plot(t_eval, angler_velocity, label='theta_dot (Angular Velocity)')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity / Angular Velocity')
    plt.legend()

    plt.tight_layout()
    plt.show()

class PendulumAnimator:
    def __init__(self, displacements: List[float], thetas: List[float], t_eval: List[float], initial_state: List[float]):
        self.displacements = displacements
        self.thetas = thetas
        self.t_eval = t_eval
        self.initial_state = initial_state
        self.t_eval_interval = t_eval[1] - t_eval[0]
        self.t_frame = 0.04

        # 动画设置
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        plt.subplots_adjust(left=0.1, bottom=0.3)
        self.cart_width, self.cart_height = 0.4, 0.2
        self.pendulum_length = 1  # 摆长度比例因子（视觉效果）
        self.ax.set_xlim([-3, 3])
        self.ax.set_ylim([-1.5, 1.5])
        self.ax.set_aspect('equal')

        # 添加表示地面的横线
        self.ax.axhline(0, color='gray', lw=2, zorder=1)

        # y轴正方向的虚线
        self.ax.axvline(0, 0.5, color='gray', linestyle='--', zorder=1)

        # 小车和倒立摆图形元素
        self.cart = plt.Rectangle((-self.cart_width / 2, -self.cart_height / 2), self.cart_width, self.cart_height, color="blue", zorder=2)
        self.ax.add_patch(self.cart)
        self.line, = self.ax.plot([], [], lw=2, color='black')  # 摆杆

        # 添加虚线表示小车和摆杆的初始位置
        self.initial_pendulum_x = self.initial_state[0] + self.pendulum_length * np.sin(self.initial_state[2])
        self.initial_pendulum_y = self.pendulum_length * np.cos(self.initial_state[2])
        self.ax.plot([self.initial_state[0], self.initial_pendulum_x], [0, self.initial_pendulum_y], 'k--', label="Initial Pendulum Position")
        self.cart.set_xy((-self.cart_width / 2, -self.cart_height / 2))
        self.line.set_data([], [])

        # 设置进度条
        self.ax_slider = plt.axes([0.2, 0.1, 0.65, 0.05], facecolor='lightgoldenrodyellow')
        self.slider = Slider(self.ax_slider, 'Time', 0, self.t_eval[-1], valinit=0, valstep=self.t_eval[1] - self.t_eval[0])

        # 播放/暂停按钮
        self.ax_play = plt.axes([0.1, 0.1, 0.05, 0.05])
        self.ax_pause = plt.axes([0.15, 0.1, 0.05, 0.05])
        self.btn_play = Button(self.ax_play, 'Play')
        self.btn_pause = Button(self.ax_pause, 'Pause')

        # 监听进度条的变化
        self.slider.on_changed(self.update_slider)

        # 监听按钮点击
        self.btn_play.on_clicked(self.play)
        self.btn_pause.on_clicked(self.pause)

        # 动画控制参数
        self.is_paused = False
        self.current_frame = 0
        self.timer = None

        # 初始化动画定时器
        self.timer = self.fig.canvas.new_timer(interval=self.t_frame*1000)
        self.timer.add_callback(self.animate)  # 指定回调函数
        self.timer.start()  # 设置定时器实现循环播放

    def update(self, frame):
        x = self.displacements[frame]
        theta = self.thetas[frame]

        # 小车位置
        self.cart.set_xy((x - self.cart_width / 2, -self.cart_height / 2))

        # 倒立摆的杆端点
        pendulum_x = x + self.pendulum_length * np.sin(theta)
        pendulum_y = self.pendulum_length * np.cos(theta)

        # 更新摆杆
        self.line.set_data([x, pendulum_x], [0, pendulum_y])

    def update_slider(self, val):
        self.current_frame = np.searchsorted(self.t_eval, val)
        self.update(self.current_frame)

    def play(self, event):
        self.is_paused = False
        if self.timer is not None:  # 如果已经有定时器在运行，则停止它
            self.timer.stop()  # 停止旧的定时器
        self.timer.start()  # 重新开始定时器

    def pause(self, event):
        self.is_paused = True

    def animate(self):
        if not self.is_paused:
            self.update(self.current_frame)
            self.slider.set_val(self.t_eval[self.current_frame])
            self.current_frame += int(self.t_frame/self.t_eval_interval)
            if self.current_frame >= len(self.t_eval):
                self.current_frame = 0
            self.fig.canvas.draw_idle()

    def start(self):
        plt.show()

def observer_error_analysis(time: List[float], error: List[List[float]]):
    displacements_error = error[0]
    angle_error = error[2]
    velocity_error = error[1]
    angler_velocity_error = error[3]
    plt.figure(figsize=(10, 8))
    plt.subplot(2, 1, 1)
    plt.plot(time, displacements_error, label='x_err (Position error)')
    plt.plot(time, angle_error, label='theta_err (Angle error)')
    plt.xlabel('Time [s]')
    plt.ylabel('Position / Angle')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time, velocity_error, label='x_dot_err (Velocity error)')
    plt.plot(time, angler_velocity_error, label='theta_dot_err (Angular Velocity error)')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity / Angular Velocity')
    plt.legend()

    plt.tight_layout()
    plt.show()