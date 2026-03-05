import numpy as np
import scipy.linalg
from uav_mpc.models.quadrotor_dynamics import QuadrotorDynamics

class LQRBaseline:
    def __init__(self, mass=1.0, Q=None, R=None):
        self.dyn = QuadrotorDynamics(mass=mass)
        self.mass = mass
        self.g = self.dyn.g
        
        # State weights: [p(3), v(3), att(3), w(3)]
        if Q is None:
            self.Q = np.diag([10, 10, 10, 1, 1, 1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01])
        else:
            self.Q = np.array(Q)
            
        # Control weights: [T, tau_x, tau_y, tau_z]
        if R is None:
            self.R = np.diag([0.1, 10, 10, 10])
        else:
            self.R = np.array(R)
            
        self._linearize_and_solve()

    def _linearize_and_solve(self):
        # Linearized dynamics around hover
        # A matrix (12x12)
        A = np.zeros((12, 12))
        A[0:3, 3:6] = np.eye(3) # p_dot = v
        A[3, 7] = -self.g      # v_x_dot = -g * theta
        A[4, 6] = self.g       # v_y_dot = g * phi
        A[6:9, 9:12] = np.eye(3) # att_dot = w
        
        # B matrix (12x4)
        B = np.zeros((12, 4))
        B[5, 0] = 1.0 / self.mass # v_z_dot = T/m (hover is already accounted, so T_cmd tracks deltas)
        invJ = np.diag([1.0/self.dyn.Ixx, 1.0/self.dyn.Iyy, 1.0/self.dyn.Izz])
        B[9:12, 1:4] = invJ
        
        # Continuous Algebraic Riccati Equation
        X = scipy.linalg.solve_continuous_are(A, B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ (B.T @ X)
        
    def solve(self, x_current, x_target):
        # LQR control law: u = -K * (x - x_target)
        # Note: LQR only predicts next instant (myopic)
        delta_x = x_current - x_target[:, 0]
        
        u_fb = -self.K @ delta_x
        
        # Feedforward hover thrust
        u_ff = np.array([self.mass * self.g, 0, 0, 0])
        
        u_opt = u_ff + u_fb
        return u_opt
