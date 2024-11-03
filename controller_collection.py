from abc import ABC, abstractmethod
import numpy as np
from scipy.signal import place_poles
from scipy.signal import cont2discrete
from scipy.linalg import solve_continuous_are
import scipy.sparse as sp
from numpy.linalg import inv
import math
import osqp

def are_floats_equal(a, b):
    tol=1e-6
    return abs(a - b) < tol

# 控制器抽象基类，确保接口统一
class Controller(ABC):

    @abstractmethod
    def control(self, t, state) -> float:
        pass

class OpenLoopController(Controller):
    def __init__(self, constant_u):
        self.constant_u = constant_u
    def control(self, t, state):
        return self.constant_u

class PolePlacementController(Controller):

    def __init__(self, desired_poles):
        self.K = None
        self.desired_poles = desired_poles

    def control(self, t, state):
        if t == 0:

            # linearlized model parameter
            matrix_A = np.array([[0, 1, 0, 0],
            [0, 0, -0.363, 0],
            [0, 0, 0, 1],
            [0, 0, 15.244, 0]])

            matrix_B = np.array([[0],
                        [0.494],
                        [0],
                        [-0.741]])
            
            placed_poles = place_poles(matrix_A, matrix_B, self.desired_poles)
            self.K = placed_poles.gain_matrix
            output = -self.K @ state
            print("Feedback matrix K:")
            print(self.K)
        else:
            output = -self.K @ state
        output = output[0]
        return output
    
class LQRController(Controller):

    def __init__(self, matrix_Q, matrix_R):
        self.matrix_Q = matrix_Q
        self.matrix_R = matrix_R
        self.K = None

    def control(self, t, state):
        if t == 0:

            # linearlized model parameter
            matrix_A = np.array([[0, 1, 0, 0],
            [0, 0, -0.363, 0],
            [0, 0, 0, 1],
            [0, 0, 15.244, 0]])

            matrix_B = np.array([[0],
                        [0.494],
                        [0],
                        [-0.741]])

            P = solve_continuous_are(matrix_A, matrix_B, self.matrix_Q, self.matrix_R)
            self.K = inv(self.matrix_R).dot(matrix_B.T).dot(P)
            output = -self.K @ state

            print("Feedback matrix K:")
            print(self.K)
        else:
            output = -self.K @ state
        output = output[0]
        return output
        
class PidHandler:

    def __init__(self, Kp, Ki, Kd, Ts):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.last_error = None
        self.integral_term = None
        self.output = 0

    def control(self, t, actual_val, target, polarity = 1.0):
        if math.isclose(t%self.Ts, 0, abs_tol=1e-6) or math.isclose(t%self.Ts, self.Ts, abs_tol=1e-6):
            error = target - actual_val
            error *= polarity
            self.output = self.Kp * error
            if t == 0 or self.last_error == None or self.integral_term == None:
                self.integral_term = error
                self.output += self.Ki * self.integral_term
            else:
                self.integral_term += error
                self.output += self.Ki * self.integral_term
                self.output += self.Kd * (error - self.last_error)
            self.last_error = error
            # print("t:%.2f actual_val:%.2f output:%.2f error%.2f" % (t, actual_val, output, error))
        return self.output

# single theta loop
class PidController1(Controller):

    def __init__(self, Kp_theta, Ki_theta, Kd_theta, desired_theta, Ts):
        self.desired_theta = desired_theta
        self.controller_theta = PidHandler(Kp_theta, Ki_theta, Kd_theta, Ts)
        self.Ts = Ts

    def control(self, t, state):
        theta = float(state[2])

        output = self.controller_theta.control(t, theta, self.desired_theta, -1.0)
        temp = t%self.Ts
        if math.isclose(t%self.Ts, 0, abs_tol=1e-6) or math.isclose(t%self.Ts, self.Ts, abs_tol=1e-6):
            print('PID: t %.2f theta  %.2f output %.2f' % (t, theta, output))
        return output

# two cascaded loop, theta, theta_dot
class PidController2(Controller):

    def __init__(self, Kp_theta, Ki_theta, Kd_theta, desired_theta \
                     , Kp_theta_dot, Ki_theta_dot, Kd_theta_dot, desired_theta_dot, Ts):
        self.desired_theta = desired_theta
        self.controller_theta = PidHandler(Kp_theta, Ki_theta, Kd_theta, Ts)
        self.desired_theta_dot = desired_theta_dot
        self.controller_theta_dot = PidHandler(Kp_theta_dot, Ki_theta_dot, Kd_theta_dot, Ts)

    def control(self, t, state):
        theta = float(state[2])
        theta_dot = float(state[3])

        output = self.controller_theta.control(t, theta, self.desired_theta) # theta in outer loop
        output = self.controller_theta_dot.control(t, theta_dot, output, -1.0) # theta_dot in inner loop
        # print('PID: t %.2f theta %.2f theta_dot %.2f output %.2f' % (t, theta, theta_dot, output))
        return output
    
# three cascaded loop, displancement, theta, theta_dot
class PidController3(Controller):

    def __init__(self, Kp_disp, Ki_disp, Kd_disp, desired_disp \
                     , Kp_theta_dot, Ki_theta_dot, Kd_theta_dot, desired_theta_dot \
                     ,Kp_theta, Ki_theta, Kd_theta, desired_theta, Ts):
        self.desired_disp = desired_disp
        self.controller_disp = PidHandler(Kp_disp, Ki_disp, Kd_disp, Ts)
        self.desired_theta = desired_theta
        self.controller_theta = PidHandler(Kp_theta, Ki_theta, Kd_theta, Ts)
        self.desired_theta_dot = desired_theta_dot
        self.controller_theta_dot = PidHandler(Kp_theta_dot, Ki_theta_dot, Kd_theta_dot, Ts)

    def control(self, t, state):
        disp = float(state[0])
        theta = float(state[2])
        theta_dot = float(state[3])

        output = self.controller_disp.control(t, disp, self.desired_disp) # displacement in the most inner loop
        print('dynamic theta target %.2f' % output)
        output = self.controller_theta.control(t, theta, output) # theta in outer loop
        print('dynamic theta_dot target %.2f' % output)
        output = self.controller_theta_dot.control(t, theta_dot, output, -1.0) # theta_dot in inner loop
        print('PID: t %.2f displacement %.2f theta %.2f theta_dot %.2f output %.2f' % (t, disp, theta, theta_dot, output))
        return output
        
class MPCController(Controller):

    def __init__(self, Ts):
        # linearlized continue model parameter
        A = np.array([[0, 1, 0, 0],
        [0, 0, -0.363, 0],
        [0, 0, 0, 1],
        [0, 0, 15.244, 0]])
        B = np.array([[0],
                    [0.494],
                    [0],
                    [-0.741]])
        C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        D = np.array([[0], [0]])
        system = (A, B, C, D)
        # get discrete system
        A_d, B_d, C_d, D_d, dt = cont2discrete(system, Ts, method='backward_diff')

        # MPC权重矩阵
        Q = np.array([[2, 0, 0, 0],
                                        [0, 0.1, 0, 0],
                                        [0, 0, 10, 0],
                                        [0, 0, 0, 0.2]])
                    
        R = np.array([[1]])
        Qf = Q  # 终端权重矩阵

        # 状态和控制输入约束
        x_min, x_max = -5, 5
        theta_min, theta_max = -30*np.pi/180, 30*np.pi/180
        u_min, u_max = -10, 10

        # MPC预测步数
        N = 500
        self.N = N

        # 构造MPC的二次规划问题
        # 构建H矩阵
        Q_block = sp.block_diag([Q]*N + [Qf])  # N 个 Q 和 1 个 Qf
        R_block = sp.block_diag([R]*N)
        H = sp.block_diag([Q_block, R_block]).tocsc()  # 组合H矩阵

        # 构造f向量
        f = np.zeros((4*(N+1) + N,))

        # 构造动态约束矩阵
        Ax = sp.kron(sp.eye(N+1), -np.eye(4)) + sp.kron(sp.eye(N+1, k=-1), A_d)
        Bu = sp.kron(sp.vstack([np.zeros((1, N)), np.eye(N)]), B_d)
        Aeq = sp.hstack([Ax, Bu])
        leq = np.zeros((4*(N+1),))
        ueq = np.zeros((4*(N+1),))

        # 构造状态和输入约束
        lx = np.tile([x_min, -np.inf, theta_min, -np.inf], N+1)
        ux = np.tile([x_max, np.inf, theta_max, np.inf], N+1)
        lu = np.tile(u_min, N)
        uu = np.tile(u_max, N)

        # 完整约束
        self.l = np.hstack([leq, lx, lu])
        self.u = np.hstack([ueq, ux, uu])

        # 使用OSQP设置问题
        self.prob = osqp.OSQP()
        self.prob.setup(P=H, q=f, A=sp.vstack([Aeq, sp.eye(4*(N+1) + N)]).tocsc(), l=self.l, u=self.u, verbose=False)

    def control(self, t, state):
        self.l[:4] = np.array(state) * -1
        self.u[:4] = np.array(state) * -1
        self.prob.update(l=self.l, u=self.u)

        # 求解优化问题
        res = self.prob.solve()
        if res.info.status != 'solved':
            print("QP问题未能解决")

        # 提取第一个控制输入并施加到系统
        output = res.x[-self.N]
        print('output %.2f' % (output))
        return output
