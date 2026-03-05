export default function TelemetryPanel({ telemetry }: { telemetry: any }) {
    return (
        <div className="metrics-panel glass-panel">
            <div className="metric-card">
                <div className="metric-header">Position Error (m)</div>
                <div className="metric-value">
                    {Math.sqrt(telemetry.x ** 2 + telemetry.y ** 2 + (telemetry.z - 1) ** 2).toFixed(3)}
                    <span className="metric-unit">RMS</span>
                </div>
            </div>

            <div className="metric-card">
                <div className="metric-header">Altitude (Z)</div>
                <div className="metric-value">
                    {telemetry.z.toFixed(2)}
                    <span className="metric-unit">m</span>
                </div>
            </div>

            <div className="metric-card">
                <div className="metric-header">Solver Latency</div>
                <div className="metric-value" style={{ color: telemetry.latency < 27 ? 'var(--accent-green)' : 'var(--accent-orange)' }}>
                    {telemetry.latency.toFixed(1)}
                    <span className="metric-unit">ms</span>
                </div>
            </div>

            <div className="metric-card">
                <div className="metric-header">Wind Disturbance</div>
                <div className="metric-value" style={{ color: 'var(--accent-cyan)' }}>
                    {telemetry.wind.toFixed(1)}
                    <span className="metric-unit">m/s</span>
                </div>
            </div>

            <div className="metric-card">
                <div className="metric-header">Attitude (deg)</div>
                <div style={{ display: 'flex', gap: '12px', marginTop: '8px' }}>
                    <div>
                        <div className="metric-unit">R</div>
                        <div style={{ fontFamily: 'var(--font-mono)' }}>{telemetry.roll.toFixed(1)}°</div>
                    </div>
                    <div>
                        <div className="metric-unit">P</div>
                        <div style={{ fontFamily: 'var(--font-mono)' }}>{telemetry.pitch.toFixed(1)}°</div>
                    </div>
                    <div>
                        <div className="metric-unit">Y</div>
                        <div style={{ fontFamily: 'var(--font-mono)' }}>{telemetry.yaw.toFixed(1)}°</div>
                    </div>
                </div>
            </div>
        </div>
    );
}
