import time
import numpy as np
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from uav_mpc.controllers.mpc_controller import MPCController
from uav_mpc.controllers.receding_horizon import RecedingHorizonController
from uav_mpc.controllers.lqr_baseline import LQRBaseline

def run_latency_benchmark(trials=50):
    print("--- Starting Latency Benchmark ---")
    
    mpc = MPCController(N=20)
    rh_mpc = RecedingHorizonController(mpc)
    lqr = LQRBaseline()
    
    x0 = np.zeros(12)
    x0[0] = 1.0 # Initial error
    X_ref = np.zeros((12, mpc.N + 1))
    
    # Warm up MPC
    rh_mpc.solve(x0, X_ref)
    
    mpc_times = []
    lqr_times = []
    
    for i in range(trials):
        # Randomize initial condition slightly
        x_curr = x0 + np.random.normal(0, 0.1, 12)
        
        # Benchmark LQR
        t0 = time.perf_counter()
        lqr.solve(x_curr, X_ref)
        lqr_times.append((time.perf_counter() - t0) * 1000)
        
        # Benchmark MPC
        t0 = time.perf_counter()
        rh_mpc.solve(x_curr, X_ref)
        mpc_times.append((time.perf_counter() - t0) * 1000)
        
    mpc_mean = np.mean(mpc_times)
    lqr_mean = np.mean(lqr_times)
    
    print(f"LQR Mean Latency: {lqr_mean:.3f} ms")
    print(f"MPC Mean Latency: {mpc_mean:.3f} ms")
    print(f"MPC Max Latency:  {np.max(mpc_times):.3f} ms")
    print(f"Target Threshold: 27.0 ms (43% reduction target)")
    
    status = "SUCCESS" if mpc_mean < 27.0 else "FAIL"
    print(f"Benchmark Status: {status}")

if __name__ == "__main__":
    run_latency_benchmark()
