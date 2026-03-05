import { Crosshair, Play, Square, Activity, Settings2 } from 'lucide-react';

interface SidebarProps {
    onRunMonteCarlo: () => void;
    onSimulateDisturbance: () => void;
    onUpdateHorizon: () => void;
    onEmergencyStop: () => void;
    horizon: number;
}

export default function Sidebar({
    onRunMonteCarlo,
    onSimulateDisturbance,
    onUpdateHorizon,
    onEmergencyStop,
    horizon
}: SidebarProps) {
    return (
        <div className="sidebar glass-panel">
            <div className="brand-header">
                <Crosshair className="brand-icon" size={28} />
                <h1 className="brand-title">UAV MISSION CTRL</h1>
            </div>

            <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
                <button className="btn btn-primary" onClick={onRunMonteCarlo}>
                    <Play size={18} /> Run Monte Carlo
                </button>
                <button className="btn btn-primary" onClick={onSimulateDisturbance}>
                    <Activity size={18} /> Simulate Disturbance
                </button>
                <button className="btn btn-primary" onClick={onUpdateHorizon}>
                    <Settings2 size={18} /> Update Horizon (N={horizon})
                </button>
            </div>

            <div style={{ marginTop: 'auto' }}>
                <button className="btn btn-danger" onClick={onEmergencyStop}>
                    <Square size={18} /> EMERGENCY STOP
                </button>
            </div>
        </div>
    );
}
