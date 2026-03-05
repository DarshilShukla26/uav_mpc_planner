import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from uav_mpc.controllers.mpc_controller import MPCController
from uav_mpc.controllers.receding_horizon import RecedingHorizonController
from evaluation.monte_carlo.wind_sampler import MonteCarloWindSampler
from uav_mpc.models.quadrotor_dynamics import QuadrotorDynamics

def simulate_trial(seed, wind_scenario, length_sec=5.0, dt=0.05):
    """
    Runs a fast headless simulation using RK4 dynamics integration 
    rather than waiting for real-time Gazebo.
    """
    mpc = MPCController(N=20, dt=dt)
    rh_mpc = RecedingHorizonController(mpc)
    dyn = QuadrotorDynamics()
    f_discrete = dyn.get_discrete_dynamics(dt)
    
    x_curr = np.zeros(12)
    x_curr[0] = 0.5 # start off target
    
    X_ref = np.zeros((12, mpc.N + 1))
    
    steps = int(length_sec / dt)
    
    pos_errors = []
    
    # Warmstart
    rh_mpc.solve(x_curr, X_ref)
    
    f_wind = wind_scenario['wind_force']
    
    for i in range(steps):
        # Add turbulence
        turb = np.random.normal(0, wind_scenario['turbulence_intensity'], 3)
        actual_wind = f_wind + turb * 0.1
        
        # MPC Solve
        U_opt, _ = rh_mpc.solve(x_curr, X_ref, f_wind_expected=f_wind) if hasattr(rh_mpc, 'f_wind_expected') else rh_mpc.solve(x_curr, X_ref, f_wind)
        u0 = U_opt[:, 0]
        
        # Step Dynamics
        x_next = np.array(f_discrete(x_curr, u0, actual_wind)).flatten()
        x_curr = x_next
        
        error = np.linalg.norm(x_curr[0:3] - X_ref[0:3, 0])
        pos_errors.append(error)
        
        # Crash condition
        if error > 2.0 or abs(x_curr[6]) > np.pi/2 or abs(x_curr[7]) > np.pi/2:
            return False, pos_errors
            
    return True, pos_errors

def run_monte_carlo(num_trials=200):
    print(f"--- Running {num_trials} Monte Carlo trials ---")
    sampler = MonteCarloWindSampler()
    
    success_count = 0
    all_errors = []
    
    t0 = time.time()
    
    for trial in range(num_trials):
        wind = sampler.sample_scenario(seed=trial)
        success, errors = simulate_trial(trial, wind)
        
        if success:
            success_count += 1
            # Usually we evaluate steady state error
            all_errors.extend(errors[-int(2.0/0.05):]) # Last 2 seconds
            
        sys.stdout.write(f"\rProgress: {trial+1}/{num_trials} (Success Rate: {success_count/(trial+1)*100:.1f}%)")
        sys.stdout.flush()
        
    print(f"\nCompleted in {time.time() - t0:.1f}s")
    
    stability = success_count / num_trials * 100
    mean_error = np.mean(all_errors) if len(all_errors) > 0 else float('inf')
    
    print("--- Monte Carlo Results ---")
    print(f"Stability (Success Rate): {stability:.1f}%")
    print(f"Mean Position Error:      {mean_error:.4f} m")
    print(f"Validation Target Error:  0.08 m")
    
    if stability >= 97.5 and mean_error <= 0.085:
        print("STATUS: PASSED VALIDATION TARGETS")
    else:
        print("STATUS: FAILED VALIDATION TARGETS - Check cost weights or constraints.")

if __name__ == "__main__":
    run_monte_carlo(num_trials=25) # Shortened for test, scale to 200 for full report
