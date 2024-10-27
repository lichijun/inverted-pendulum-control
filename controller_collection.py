from abc import ABC, abstractmethod
import numpy as np
from scipy.signal import place_poles
from scipy.linalg import solve_continuous_are
from numpy.linalg import inv
import math

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
        
class PidController(Controller):

    def __init__(self, Kp_theta, Ki_theta, Kd_theta, Ts_theta, desired_theta \
                     , Kp_disp, Ki_disp, Kd_disp, Ts_disp, desired_disp):
        self.desired_theta = desired_theta
        self.controller_theta = PidHandler(Kp_theta, Ki_theta, Kd_theta, Ts_theta)
        self.desired_disp = desired_disp
        self.controller_disp = PidHandler(Kp_disp, Ki_disp, Kd_disp, Ts_disp)

    def control(self, t, state):
        theta = float(state[2])
        theta_dot = float(state[3])

        # # single PID, theta
        # output = self.controller_theta.control(t, theta, self.desired_theta, -1.0)

        # # parallel PID, not feasible because the angle and position are coupled
        # output1 = self.controller_theta.control(t, theta, self.desired_theta, -1.0)
        # output2 = self.controller_disp.control(t, disp, self.desired_disp)
        # output = output1 + output2

        # cascaded PID, theta in outer loop, theta_dot in inner loop
        output = self.controller_theta.control(t, theta, self.desired_theta)
        output = self.controller_disp.control(t, theta_dot, output, -1.0)
        print('PID: t %.2f theta %.2f theta_dot %.2f output %.2f' % (t, theta, theta_dot, output))
        return output
