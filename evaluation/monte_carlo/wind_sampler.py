import numpy as np

class MonteCarloWindSampler:
    def __init__(self, max_mean_wind=8.0, max_turbulence=2.0):
        self.max_wind = max_mean_wind
        self.max_turb = max_turbulence
        
    def sample_scenario(self, seed=None):
        if seed is not None:
            np.random.seed(seed)
            
        # Draw random heading and magnitude for constant wind
        angle = np.random.uniform(0, 2 * np.pi)
        mag = np.random.uniform(0, self.max_wind)
        wx = mag * np.cos(angle)
        wy = mag * np.sin(angle)
        wz = np.random.uniform(-1.0, 1.0) # Light updraft/downdraft
        
        turb_intensity = np.random.uniform(0, self.max_turb)
        
        return {
            'mean_wind': np.array([wx, wy, wz]),
            'turbulence_intensity': turb_intensity,
            'wind_force': np.array([wx, wy, wz]) * 0.1 # Example force coefficient
        }
