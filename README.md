# UAV MPC Planner

A high-performance nonlinear Model Predictive Control (MPC) planner for a 6-DOF Quadrotor, utilizing CasADi for formulation and Pybind11 / C++ code generation constraints.

## Architecture

* **Dynamics (`uav_mpc/models/quadrotor_dynamics.py`)**: 12-state rigorous Euler-angle based rigid body dynamics model. Incorporates wind turbulence via the internal ODE structure.
* **Controller (`uav_mpc/controllers/mpc_controller.py`)**: 20-step nonlinear optimal control problem defined via CasADi `Opti`. Generates underlying C code or utilizes internal IPOPT JIT optimized evaluations to meet the 50Hz control timeline.
* **Nodes (`uav_mpc/ros2_nodes`)**: High level execution environment configured for ROS2 (Humble/Iron) using publishers and subscribers for telemetry and visualization.

## Setup & Dependencies

```bash
# Python Requirements
pip install casadi pybind11 cmake numpy pytest scipy

# Build C++ Binder (If active in RecedingHorizon properties)
cd cpp_solver && mkdir -p build && cd build
cmake -DCMAKE_PREFIX_PATH=$(python3 -c "import pybind11; print(pybind11.get_cmake_dir())") ..
make -j4
```

## Running Benchmarking & Validation

We enforce aggressive validation frameworks on both computational complexity (Latency) and mathematical robustness (Wind Disturbance Monte Carlo).

```bash
# Verify unit tests (Jacobians, Math Definitions)
pytest tests/

# Measure MPC computing latency against an LQR baseline
python3 evaluation/benchmarks/latency_benchmark.py

# Test robustness against stochastic wind loads (200 trials)
python3 evaluation/monte_carlo/run_monte_carlo.py
```

## Design Decisions
- Reverted full standalone SQP native C emission mapping due to macOS static linking constraints for Casadi Opti/`mpc_solver_func` memory allocation issues. Fallbacked seamlessly to Casadi `JIT=True, O3` optimization over IPOPT yielding natively optimized execution logic comfortably hitting `<20ms` latency scaling per iteration.
- Enforced limits `max_iter = 10` paired with `warm_start_init_point` to exploit the temporal continuity over the 20-step predictive receding window.

## Tags
Reproducible Release internally labeled.
