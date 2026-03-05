import casadi as ca
import numpy as np

class QuadrotorDynamics:
    def __init__(self, mass=1.0, g=9.81, Ixx=0.01, Iyy=0.01, Izz=0.02):
        self.mass = mass
        self.g = g
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.J = ca.diag([Ixx, Iyy, Izz])
        self.invJ = ca.diag([1.0/Ixx, 1.0/Iyy, 1.0/Izz])

        self._setup_dynamics()

    def _setup_dynamics(self):
        # State variables (12)
        # Position
        px = ca.SX.sym('px')
        py = ca.SX.sym('py')
        pz = ca.SX.sym('pz')
        p = ca.vertcat(px, py, pz)

        # Velocity
        vx = ca.SX.sym('vx')
        vy = ca.SX.sym('vy')
        vz = ca.SX.sym('vz')
        v = ca.vertcat(vx, vy, vz)

        # Attitude (Euler angles: roll, pitch, yaw)
        phi = ca.SX.sym('phi')
        theta = ca.SX.sym('theta')
        psi = ca.SX.sym('psi')
        att = ca.vertcat(phi, theta, psi)

        # Angular rates (body frame)
        p_rate = ca.SX.sym('p_rate')
        q_rate = ca.SX.sym('q_rate')
        r_rate = ca.SX.sym('r_rate')
        omega = ca.vertcat(p_rate, q_rate, r_rate)

        self.x = ca.vertcat(p, v, att, omega)
        self.nx = self.x.shape[0]

        # Control variables (4)
        T = ca.SX.sym('T')  # Total thrust
        tau_x = ca.SX.sym('tau_x') # Roll torque
        tau_y = ca.SX.sym('tau_y') # Pitch torque
        tau_z = ca.SX.sym('tau_z') # Yaw torque
        self.u = ca.vertcat(T, tau_x, tau_y, tau_z)
        self.nu = self.u.shape[0]

        # External disturbance (wind force)
        fw_x = ca.SX.sym('fw_x')
        fw_y = ca.SX.sym('fw_y')
        fw_z = ca.SX.sym('fw_z')
        self.f_wind = ca.vertcat(fw_x, fw_y, fw_z)

        # Rotation matrix from Body to World frame (Z-Y-X Euler angles)
        R_x = ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, ca.cos(phi), -ca.sin(phi)),
            ca.horzcat(0, ca.sin(phi), ca.cos(phi))
        )
        R_y = ca.vertcat(
            ca.horzcat(ca.cos(theta), 0, ca.sin(theta)),
            ca.horzcat(0, 1, 0),
            ca.horzcat(-ca.sin(theta), 0, ca.cos(theta))
        )
        R_z = ca.vertcat(
            ca.horzcat(ca.cos(psi), -ca.sin(psi), 0),
            ca.horzcat(ca.sin(psi), ca.cos(psi), 0),
            ca.horzcat(0, 0, 1)
        )
        R = R_z @ R_y @ R_x

        # Drone frame aligns Z up. Control pushes along local Z.
        # F_thrust vector in world frame
        thrust_world = R @ ca.vertcat(0, 0, T)
        
        # Gravity vector in world frame
        gravity = ca.vertcat(0, 0, -self.mass * self.g)

        # Translational dynamics
        p_dot = v
        v_dot = (gravity + thrust_world + self.f_wind) / self.mass

        # Rotational kinematics (Euler angles rates from body rates)
        # W maps body rates to Euler angle rates
        W_inv = ca.vertcat(
            ca.horzcat(1, ca.sin(phi)*ca.tan(theta), ca.cos(phi)*ca.tan(theta)),
            ca.horzcat(0, ca.cos(phi), -ca.sin(phi)),
            ca.horzcat(0, ca.sin(phi)/ca.cos(theta), ca.cos(phi)/ca.cos(theta))
        )
        att_dot = W_inv @ omega

        # Rotational dynamics (Euler's equations)
        tau = ca.vertcat(tau_x, tau_y, tau_z)
        # omega_dot = J_inv * (tau - cross(omega, J * omega))
        # casadi cross function
        omega_cross = ca.cross(omega, self.J @ omega)
        omega_dot = self.invJ @ (tau - omega_cross)

        # Full state derivative
        self.x_dot = ca.vertcat(p_dot, v_dot, att_dot, omega_dot)

        # Create CasADi function for continuous ODE
        self.f_ode = ca.Function('f_ode', [self.x, self.u, self.f_wind], [self.x_dot], ['x', 'u', 'f_wind'], ['x_dot'])

    def get_discrete_dynamics(self, dt, method='rk4'):
        """ Returns a CasADi function for discrete time dynamics. """
        if method == 'euler':
            x_next = self.x + dt * self.f_ode(self.x, self.u, self.f_wind)
        elif method == 'rk4':
            k1 = self.f_ode(self.x, self.u, self.f_wind)
            k2 = self.f_ode(self.x + dt / 2.0 * k1, self.u, self.f_wind)
            k3 = self.f_ode(self.x + dt / 2.0 * k2, self.u, self.f_wind)
            k4 = self.f_ode(self.x + dt * k3, self.u, self.f_wind)
            x_next = self.x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        else:
            raise ValueError("Unsupported integration method")

        return ca.Function('f_discrete', [self.x, self.u, self.f_wind], [x_next], ['x', 'u', 'f_wind'], ['x_next'])

