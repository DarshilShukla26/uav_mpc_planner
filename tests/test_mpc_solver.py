import pytest
import numpy as np
import casadi as ca
import os
import sys

# Add parent directory to path to import uav_mpc
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from uav_mpc.controllers.mpc_controller import MPCController
from uav_mpc.controllers.receding_horizon import RecedingHorizonController

def test_mpc_formulation():
    mpc = MPCController(N=10)
    assert mpc.N == 10
    
    # Try solving a simple hover problem
    x0 = np.zeros(12)
    # Give a small z error
    x0[2] = -1.0 
    
    X_ref = np.zeros((12, mpc.N + 1))
    f_wind = np.zeros(3)
    obs_params = np.zeros((4, mpc.n_obs))
    
    rh = RecedingHorizonController(mpc)
    
    U_opt, X_opt = rh.solve(x0, X_ref, f_wind, obs_params)
    
    assert U_opt.shape == (4, mpc.N)
    assert X_opt.shape == (12, mpc.N + 1)
    
    # The drone should push up (Z position is negative)
    # Expected thrust > mg
    mg = mpc.dyn.mass * mpc.dyn.g
    assert U_opt[0, 0] > mg

def test_obstacle_avoidance():
    mpc = MPCController(N=10)
    
    x0 = np.zeros(12)
    # Moving from (0,0) to (2,0)
    X_ref = np.zeros((12, mpc.N + 1))
    for k in range(mpc.N + 1):
        X_ref[0, k] = 2.0 * k / mpc.N
        
    f_wind = np.zeros(3)
    obs_params = np.zeros((4, mpc.n_obs))
    # Place obstacle at (1, 0, 0) with radius 0.5
    obs_params[0, 0] = 1.0
    obs_params[1, 0] = 0.0
    obs_params[2, 0] = 0.0
    obs_params[3, 0] = 0.5
    
    rh = RecedingHorizonController(mpc)
    U_opt, X_opt = rh.solve(x0, X_ref, f_wind, obs_params)
    
    # Check if the drone stayed away from (1,0,0)
    # Find minimum distance to (1,0,0) among X_opt trajectory
    min_dist_sq = np.min((X_opt[0, :] - 1.0)**2 + (X_opt[1, :])**2 + (X_opt[2, :])**2)
    assert min_dist_sq > 0.5**2
