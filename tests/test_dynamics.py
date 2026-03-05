import pytest
import numpy as np
import casadi as ca
import os
import sys

# Add parent directory to path to import uav_mpc
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from uav_mpc.models.quadrotor_dynamics import QuadrotorDynamics
from uav_mpc.models.wind_disturbance import WindDisturbance

def test_hover_dynamics():
    # Test that exactly matching gravity with thrust results in zero acceleration
    mass = 1.0
    g = 9.81
    dyn = QuadrotorDynamics(mass=mass, g=g)
    
    # State = zeros
    x0 = np.zeros(12)
    # Control: Thrust mg, torques 0
    u_hover = np.array([mass * g, 0.0, 0.0, 0.0])
    f_wind = np.zeros(3)
    
    x_dot = dyn.f_ode(x0, u_hover, f_wind)
    x_dot_np = np.array(x_dot).flatten()
    
    # Assert all derivatives are very close to zero
    np.testing.assert_allclose(x_dot_np, np.zeros(12), atol=1e-8)

def test_translational_dynamics():
    # Test upward acceleration
    mass = 1.0
    g = 9.81
    dyn = QuadrotorDynamics(mass=mass, g=g)
    
    x0 = np.zeros(12)
    u_accel = np.array([mass * g + 2.0, 0.0, 0.0, 0.0]) # 2N extra thrust
    f_wind = np.zeros(3)
    
    x_dot = dyn.f_ode(x0, u_accel, f_wind)
    x_dot_np = np.array(x_dot).flatten()
    
    # v_z dot (index 5) should be 2.0 / mass = 2.0
    assert np.isclose(x_dot_np[5], 2.0)
    
def test_rotational_dynamics():
    mass = 1.0
    Ixx = 0.01
    dyn = QuadrotorDynamics(mass=mass, Ixx=Ixx)
    
    x0 = np.zeros(12)
    # Apply roll torque
    u_roll = np.array([mass * 9.81, 0.05, 0.0, 0.0])
    f_wind = np.zeros(3)
    
    x_dot = dyn.f_ode(x0, u_roll, f_wind)
    x_dot_np = np.array(x_dot).flatten()
    
    # p_rate dot (index 9) should be tau_x / Ixx = 0.05 / 0.01 = 5.0
    assert np.isclose(x_dot_np[9], 5.0)

def test_discrete_dynamics():
    dyn = QuadrotorDynamics()
    dt = 0.1
    f_rk4 = dyn.get_discrete_dynamics(dt, method='rk4')
    
    x0 = np.zeros(12)
    # Give it some initial velocity in x
    x0[3] = 1.0
    
    u_hover = np.array([1.0 * 9.81, 0.0, 0.0, 0.0])
    f_wind = np.zeros(3)
    
    x_next = f_rk4(x0, u_hover, f_wind)
    x_next_np = np.array(x_next).flatten()
    
    # px should have advanced by approx vx * dt = 1.0 * 0.1 = 0.1
    assert np.isclose(x_next_np[0], 0.1, atol=1e-2)

def test_wind_model():
    wind = WindDisturbance(mean_wind_velocity=[5.0, 0.0, 0.0], turbulence_intensity=0.0)
    force = wind.get_nominal_wind_force()
    assert force[0] > 0.0
    assert force[1] == 0.0
    assert force[2] == 0.0
