import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Line, Box, Sphere } from '@react-three/drei';
import * as THREE from 'three';

// Props to inject live state from the parent app
interface UAVProps {
    position: [number, number, number];
    rotation: [number, number, number];
}

function DroneModel({ position, rotation }: UAVProps) {
    const group = useRef<THREE.Group>(null);

    // Smoothly interpolate towards the target position for 60fps rendering 
    // even if the physics step (50Hz) is slightly offset
    useFrame((_state, _delta) => {
        if (group.current) {
            group.current.position.lerp(new THREE.Vector3(...position), 0.1);

            const targetQuat = new THREE.Quaternion().setFromEuler(
                new THREE.Euler(rotation[0], rotation[1], rotation[2], 'XYZ')
            );
            group.current.quaternion.slerp(targetQuat, 0.1);
        }
    });

    return (
        <group ref={group}>
            {/* Central Body */}
            <Box args={[0.2, 0.05, 0.2]}>
                <meshStandardMaterial color="#b537f2" emissive="#b537f2" emissiveIntensity={0.5} wireframe />
            </Box>

            {/* Arms & Motor Mounts */}
            <Box position={[0.15, 0, 0.15]} args={[0.02, 0.02, 0.2]} rotation={[0, Math.PI / 4, 0]}>
                <meshStandardMaterial color="#8b9bb4" />
            </Box>
            <Box position={[-0.15, 0, 0.15]} args={[0.02, 0.02, 0.2]} rotation={[0, -Math.PI / 4, 0]}>
                <meshStandardMaterial color="#8b9bb4" />
            </Box>
            <Box position={[0.15, 0, -0.15]} args={[0.02, 0.02, 0.2]} rotation={[0, -Math.PI / 4, 0]}>
                <meshStandardMaterial color="#8b9bb4" />
            </Box>
            <Box position={[-0.15, 0, -0.15]} args={[0.02, 0.02, 0.2]} rotation={[0, Math.PI / 4, 0]}>
                <meshStandardMaterial color="#8b9bb4" />
            </Box>

            {/* Propeller Disks (spinning animation simplified) */}
            {[
                [0.2, 0.05, 0.2],
                [-0.2, 0.05, 0.2],
                [0.2, 0.05, -0.2],
                [-0.2, 0.05, -0.2]
            ].map((pos, i) => (
                <Sphere key={i} args={[0.1, 16, 16]} position={pos as [number, number, number]} scale={[1, 0.1, 1]}>
                    <meshStandardMaterial color="#00f0ff" transparent opacity={0.3} emissive="#00f0ff" emissiveIntensity={0.8} />
                </Sphere>
            ))}
        </group>
    );
}

export default function UAVCanvas({ telemetry }: { telemetry: any }) {
    // Generate a mock history trail
    const points = React.useMemo(() => {
        const pts = [];
        for (let i = 0; i < 50; i++) {
            pts.push(new THREE.Vector3(
                Math.sin(i * 0.1) * 2,
                Math.cos(i * 0.1) * 2,
                1.0 + Math.sin(i * 0.2) * 0.2
            ));
        }
        return pts;
    }, []);

    return (
        <>
            <color attach="background" args={['#0a0e17']} />

            {/* Cyberpunk Lighting */}
            <ambientLight intensity={0.2} />
            <directionalLight position={[10, 10, 10]} intensity={1} color="#ffffff" />
            <pointLight position={[0, 2, 0]} intensity={2} color="#00f0ff" distance={10} />
            <pointLight position={[2, 0, -2]} intensity={2} color="#b537f2" distance={10} />

            {/* Origin Grid */}
            <Grid
                renderOrder={-1}
                position={[0, 0, 0]}
                infiniteGrid
                cellSize={1}
                cellThickness={0.5}
                sectionSize={5}
                sectionThickness={1}
                sectionColor="#4a5b78"
                fadeDistance={30}
            />

            {/* The Drone */}
            <DroneModel
                position={[telemetry.x, telemetry.z, -telemetry.y]} // Map NED/ROS coordinates to WebGL (Y-up)
                rotation={[telemetry.pitch, telemetry.yaw, telemetry.roll]}
            />

            {/* Reference MPC Path (Mocked trace) */}
            <Line
                points={points}
                color="#00ff66"
                lineWidth={2}
                transparent
                opacity={0.5}
                dashed
                dashScale={5}
            />

            {/* Next target Waypoint */}
            <Sphere args={[0.08]} position={[Math.sin(5) * 2, 1.0, -Math.cos(5) * 2]}>
                <meshBasicMaterial color="#ff3b30" wireframe />
            </Sphere>

            <OrbitControls
                makeDefault
                autoRotate
                autoRotateSpeed={0.5}
                maxPolarAngle={Math.PI / 2 - 0.05} // Prevent going below ground
            />
        </>
    );
}
