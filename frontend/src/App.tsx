import { useState, useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import Sidebar from './components/Sidebar';
import TelemetryPanel from './components/TelemetryPanel';
import UAVCanvas from './components/UAVCanvas';

function App() {
  // Hoist the telemetry mock here to feed both panels and the 3D scene
  const [telemetry, setTelemetry] = useState({
    x: 0, y: 0, z: 1.0,
    roll: 0, pitch: 0, yaw: 0,
    latency: 24.5, wind: 2.1
  });

  useEffect(() => {
    let t = 0;
    const interval = setInterval(() => {
      t += 0.05; // Simulate flight advancing
      setTelemetry({
        x: Math.sin(t) * 2, // Orbiting path
        y: Math.cos(t) * 2,
        z: 1.0 + Math.sin(t * 2) * 0.2, // Bobbing altitude

        // Tilt into the curve
        roll: Math.cos(t) * 15,
        pitch: Math.sin(t) * 15,
        yaw: -t * (180 / Math.PI),

        latency: 24.0 + Math.random() * 2.5,
        wind: 2.0 + Math.random() * 0.5
      });
    }, 50); // Fast update for smoothness
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="dashboard-layout">
      {/* Left Sidebar Tools */}
      <Sidebar />

      {/* Center 3D Visualization */}
      <div className="canvas-container">
        <div className="status-overlay">
          <div className="status-dot"></div>
          MPC SOLVER ONLINE (50Hz)
        </div>
        <Canvas camera={{ position: [5, 5, 5], fov: 45 }}>
          <UAVCanvas telemetry={telemetry} />
        </Canvas>
      </div>

      {/* Right Metrics Drawer */}
      <TelemetryPanel telemetry={telemetry} />
    </div>
  );
}

export default App;
