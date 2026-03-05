import { Crosshair, Play, Square, Activity, Settings2 } from 'lucide-react';

export default function Sidebar() {
    return (
        <div className="sidebar glass-panel">
            <div className="brand-header">
                <Crosshair className="brand-icon" size={28} />
                <h1 className="brand-title">UAV MISSION CTRL</h1>
            </div>

            <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
                <button className="btn btn-primary">
                    <Play size={18} /> Run Monte Carlo
                </button>
                <button className="btn btn-primary">
                    <Activity size={18} /> Simulate Disturbance
                </button>
                <button className="btn btn-primary">
                    <Settings2 size={18} /> Update Horizon (N=20)
                </button>
            </div>

            <div style={{ marginTop: 'auto' }}>
                <button className="btn btn-danger">
                    <Square size={18} /> EMERGENCY STOP
                </button>
            </div>
        </div>
    );
}
