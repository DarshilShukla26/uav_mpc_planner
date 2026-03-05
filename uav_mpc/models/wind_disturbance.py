import numpy as np

class WindDisturbance:
    def __init__(self, mean_wind_velocity=np.array([0., 0., 0.]), turbulence_intensity=0.0):
        """
        Wind disturbance model approximating a simplified Dryden/von Karman wind turbulence.
        For MPC planning purposes, it typically provides the expected wind force.
        
        Args:
            mean_wind_velocity: Base wind velocity vector [wx, wy, wz] (m/s)
            turbulence_intensity: Variance modifier for random gusts
        """
        self.mean_wind = np.array(mean_wind_velocity, dtype=float)
        self.intensity = turbulence_intensity
        # A simple linear drag coefficient for approximation F = 0.5 * rho * v^2 * Cd * A
        # Since this acts as a direct force disturbance on the dynamics, we abstract this
        # simply as a direct force vector.
        self.drag_coefficient_approx = 0.1 # N/(m/s)

    def sample_wind_force(self):
        """
        Generates a sample wind force. In the CasADi ODE, this adds to gravity & thrust.
        """
        if self.intensity > 1e-6:
            gust = np.random.normal(0, self.intensity, 3)
        else:
            gust = np.zeros(3)
            
        wind_vel = self.mean_wind + gust
        # F_wind approximate
        force = self.drag_coefficient_approx * wind_vel
        return force

    def get_nominal_wind_force(self):
        """
        Returns the expected (mean) wind force, useful for MPC prediction horizon.
        """
        return self.drag_coefficient_approx * self.mean_wind
