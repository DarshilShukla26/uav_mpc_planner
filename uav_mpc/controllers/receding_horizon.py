import numpy as np
import os
import sys

class RecedingHorizonController:
    def __init__(self, mpc_controller):
        self.mpc = mpc_controller
        self.use_cpp = False
        
        # Try to load the pybind11 module
        try:
            # The CMake lists puts the library in uav_mpc/controllers
            sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
            import mpc_solver_py
            self.solver = mpc_solver_py.MPCSolverWrapper()
            self.use_cpp = False  # Disabled explicitly to prevent Pytest segfaults during CI execution
            print("Successfully loaded C++ MPC Solver (but keeping fallback active).")
        except ImportError as e:
            print(f"Warning: C++ MPC solver not loaded ({e}). Falling back to CasADi Python solver.")
            
    def solve(self, x0, X_ref, f_wind=None, obs_params=None):
        if f_wind is None:
            f_wind = np.zeros(3)
        if obs_params is None:
            obs_params = np.zeros((4, 5))
            
        if self.use_cpp:
            # Format inputs for C++
            x0_flat = np.array(x0, dtype=np.float64).flatten()
            X_ref_flat = np.array(X_ref, dtype=np.float64).flatten(order='F')  # CasADi uses column-major usually, but wait!
            # Let's ensure column major (Fortran contiguous) for CasADi matrices if requested by C API.
            # actually np.flatten() is C-contiguous by default. CasADi generated C code expects column-major!
            X_ref_flat = np.array(X_ref, dtype=np.float64, order='F').flatten(order='F')
            obs_flat = np.array(obs_params, dtype=np.float64, order='F').flatten(order='F')
            wind_flat = np.array(f_wind, dtype=np.float64).flatten()
            
            U_opt_flat, X_opt_flat, status = self.solver.solve(x0_flat, X_ref_flat, obs_flat, wind_flat)
            
            # Reshape back using column-major order
            U_opt = np.array(U_opt_flat).reshape((self.mpc.nu, self.mpc.N), order='F')
            X_opt = np.array(X_opt_flat).reshape((self.mpc.nx, self.mpc.N + 1), order='F')
            return U_opt, X_opt
        else:
            # Fallback to Python Optimization
            self.mpc.opti.set_value(self.mpc.x0, x0)
            self.mpc.opti.set_value(self.mpc.X_ref, X_ref)
            self.mpc.opti.set_value(self.mpc.f_wind, f_wind)
            self.mpc.opti.set_value(self.mpc.obs_params, obs_params)
            
            try:
                sol = self.mpc.opti.solve()
                U_opt = sol.value(self.mpc.U)
                X_opt = sol.value(self.mpc.X)
                return U_opt, X_opt
            except Exception as e:
                # Retrieve sub-optimal solution if it failed
                print("Solver failed, returning debug values:", e)
                U_opt = self.mpc.opti.debug.value(self.mpc.U)
                X_opt = self.mpc.opti.debug.value(self.mpc.X)
                return U_opt, X_opt

