import casadi as ca
import numpy as np
import os

from uav_mpc.models.quadrotor_dynamics import QuadrotorDynamics

class MPCController:
    def __init__(self, N=20, dt=0.05, mass=1.0, Q=None, R=None):
        self.N = N  # Prediction horizon
        self.dt = dt
        self.dyn = QuadrotorDynamics(mass=mass)
        self.nx = self.dyn.nx
        self.nu = self.dyn.nu
        
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
            
        self._setup_opti()

    def _setup_opti(self):
        """Sets up the CasADi Opti environment for the MPC problem."""
        self.opti = ca.Opti()

        # Decision variables
        self.X = self.opti.variable(self.nx, self.N + 1) # States
        self.U = self.opti.variable(self.nu, self.N)     # Controls

        # Parameters (Inputs to the solver)
        self.x0 = self.opti.parameter(self.nx, 1)        # Initial state
        self.X_ref = self.opti.parameter(self.nx, self.N + 1) # Reference trajectory
        
        # Static obstacles (x, y, z, radius)
        # Assuming maximum 5 static obstacles for prediction
        self.n_obs = 5
        self.obs_params = self.opti.parameter(4, self.n_obs)

        # Wind parameter
        self.f_wind = self.opti.parameter(3, 1)

        # Objective and Constraints
        obj = 0

        # Discrete dynamics via RK4
        f_rk4 = self.dyn.get_discrete_dynamics(self.dt, method='rk4')

        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.x0)

        for k in range(self.N):
            # State error
            err_x = self.X[:, k] - self.X_ref[:, k]
            # Cost accumulation
            obj += ca.mtimes([err_x.T, self.Q, err_x])
            # Control usage cost (penalize usage, especially torque)
            u_hover = ca.vertcat(self.dyn.mass * self.dyn.g, 0, 0, 0)
            err_u = self.U[:, k] - u_hover
            obj += ca.mtimes([err_u.T, self.R, err_u])

            # Multiple shooting constraint (Dynamics)
            x_next = f_rk4(self.X[:, k], self.U[:, k], self.f_wind)
            self.opti.subject_to(self.X[:, k+1] == x_next)
            
            # Control constraints
            T_min = 0.1 * self.dyn.mass * self.dyn.g
            T_max = 2.0 * self.dyn.mass * self.dyn.g
            tau_max = 0.1
            self.opti.subject_to(self.opti.bounded(T_min, self.U[0, k], T_max))
            self.opti.subject_to(self.opti.bounded(-tau_max, self.U[1, k], tau_max))
            self.opti.subject_to(self.opti.bounded(-tau_max, self.U[2, k], tau_max))
            self.opti.subject_to(self.opti.bounded(-tau_max, self.U[3, k], tau_max))
            
            # Attitude constraints (prevent flipping)
            angle_max = ca.pi / 3  # 60 degrees
            self.opti.subject_to(self.opti.bounded(-angle_max, self.X[6, k], angle_max))  # phi
            self.opti.subject_to(self.opti.bounded(-angle_max, self.X[7, k], angle_max))  # theta
            
            # Obstacle avoidance logic
            # distance^2 >= (r_obs + r_drone)^2
            r_drone = 0.3 # default margin
            for obs_idx in range(self.n_obs):
                obs_pos = self.obs_params[0:3, obs_idx]
                obs_r = self.obs_params[3, obs_idx]
                
                # If obs_r is <= 0, we can ignore this obstacle (dummy obstacle)
                # We add a small smoothing factor
                dist_sq = ca.sumsqr(self.X[0:3, k] - obs_pos)
                safe_dist_sq = (obs_r + r_drone)**2
                
                # Using an IF statement in CasADi requires smoothing, but since
                # we are parameterizing it, a simple continuous relaxation helps,
                # or we just enforce it only when radius > 0.
                # A relaxed formulation:
                # We only want constraint active if obs is active (r > 0).
                # (dist_sq - safe_dist_sq) * (obs_r) >= 0 gives bad scaling.
                # For static parameters, IPOPT handles simple constraints fine:
                # To avoiding issues with absent obstacles, standard approach is:
                padding = ca.if_else(obs_r > 0.01, safe_dist_sq, 0.0)
                self.opti.subject_to(dist_sq >= padding)

        # Terminal cost
        err_x_N = self.X[:, self.N] - self.X_ref[:, self.N]
        obj += ca.mtimes([err_x_N.T, self.Q * 2.0, err_x_N])

        self.opti.minimize(obj)
        
        # IPOPT plugin options for fast solving
        p_opts = {
            "expand": True, 
            "jit": True, 
            "compiler": "shell", 
            "jit_options": {"flags": ["-O3"]}
        }
        s_opts = {"max_iter": 10, 
                  "print_level": 0, 
                  "tol": 1e-3, 
                  "acceptable_tol": 1e-2, 
                  "constr_viol_tol": 1e-2,
                  "linear_solver": "mumps",  # MA27 is faster if available
                  "warm_start_init_point": "yes"}
        
        self.opti.solver("ipopt", p_opts, s_opts)

    def generate_c_code(self, filename="mpc_solver.c", directory="cpp_solver/src/"):
        """Generates highly optimized C code for the CasADi function."""
        if not os.path.exists(directory):
            os.makedirs(directory)
            
        print(f"Generating C code to {directory}/{filename} ...")
        # Opti compiles to a parametric function
        # Map parameters to inputs, variable to outputs
        # Parameters: x0, X_ref, obs_params, f_wind
        M_args = [self.x0, self.X_ref, self.obs_params, self.f_wind]
        # Variables: primary interest is optimal U, but returning X is nice for plotting
        M_out = [self.U, self.X]
        
        solver_func = self.opti.to_function('mpc_solver_func', M_args, M_out, 
                                            ['x0', 'X_ref', 'obs_params', 'f_wind'], 
                                            ['U_opt', 'X_opt'])
        
        opts = dict(main=False, mex=False, with_header=True)
        # C-code generation
        gen_path = os.path.join(directory, filename)
        solver_func.generate(filename, opts)
        
        # move generated file
        if os.path.exists(filename):
             os.rename(filename, gen_path)
             # Casadi generates the header in the current directory as well
             if os.path.exists(filename.replace('.c', '.h')):
                 # Moving header to include directory
                 include_dir = directory.replace("src", "include").replace("src/", "include/")
                 os.rename(filename.replace('.c', '.h'), os.path.join(include_dir, filename.replace('.c', '.h')))
        
        print("C code generation successful.")

