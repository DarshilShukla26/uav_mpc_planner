import { useState, useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import Sidebar from './components/Sidebar';
import TelemetryPanel from './components/TelemetryPanel';
import UAVCanvas from './components/UAVCanvas';

function App() {
  const [isRunning, setIsRunning] = useState(true);
  const [horizon, setHorizon] = useState(20);
  const [_time, setTime] = useState(0);

  // Hoist the telemetry mock here to feed both panels and the 3D scene
  const [telemetry, setTelemetry] = useState({
    x: 0, y: 0, z: 1.0,
    roll: 0, pitch: 0, yaw: 0,
    latency: 24.5, wind: 2.1
  });

  useEffect(() => {
    if (!isRunning) return;

    const interval = setInterval(() => {
      setTime(prev => {
        const t = prev + 0.05; // Simulate flight advancing

        setTelemetry(curr => {
          // Slowly decay severe wind gusts back towards normal range
          const nextWind = Math.max(2.1, curr.wind * 0.95 + Math.random() * 0.2);

          // Introduce some positional jitter scaled by wind
          const jitter = (Math.random() - 0.5) * (nextWind * 0.02);

          return {
            x: Math.sin(t) * 2 + jitter, // Orbiting path
            y: Math.cos(t) * 2 + jitter,
            z: 1.0 + Math.sin(t * 2) * 0.2 + jitter, // Bobbing altitude

            // Tilt into the curve + wind jitter
            roll: Math.cos(t) * 15 + jitter * 10,
            pitch: Math.sin(t) * 15 + jitter * 10,
            yaw: -t * (180 / Math.PI),

            latency: 24.0 + Math.random() * 2.5,
            wind: nextWind
          };
        });

        return t;
      });
    }, 50); // Fast update for smoothness
    return () => clearInterval(interval);
  }, [isRunning]);

  const handleSimulateDisturbance = () => {
    // Inject huge spike in wind to emulate a sudden turbulent gust
    setTelemetry(curr => ({ ...curr, wind: 25.0 }));
  };

  const handleEmergencyStop = () => {
    setIsRunning(false);
    // Cut powers: drop z to 0 and level the drone
    setTelemetry(curr => ({
      ...curr,
      z: 0.0, roll: 0, pitch: 0, latency: 0, wind: 0
    }));
  };

  const handleRunMonteCarlo = () => {
    // Reset trajectory clock and start running again
    setTime(0);
    setIsRunning(true);
  };

  const handleUpdateHorizon = () => {
    setHorizon(prev => prev === 20 ? 40 : 20);
  };

  return (
    <div className="dashboard-layout">
      {/* Left Sidebar Tools */}
      <Sidebar
        onRunMonteCarlo={handleRunMonteCarlo}
        onSimulateDisturbance={handleSimulateDisturbance}
        onUpdateHorizon={handleUpdateHorizon}
        onEmergencyStop={handleEmergencyStop}
        horizon={horizon}
      />

      {/* Center 3D Visualization */}
      <div className="canvas-container">
        <div className="status-overlay">
          <div className="status-dot" style={{ animation: isRunning ? 'pulse 2s infinite' : 'none', background: isRunning ? 'var(--accent-green)' : 'var(--accent-orange)' }}></div>
          {isRunning ? "MPC SOLVER ONLINE (50Hz)" : "SYSTEM ESTOP / GROUNDED"}
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
