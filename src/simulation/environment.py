import numpy as np
from typing import List, Dict, Optional
from dataclasses import dataclass

from src.models.quadrotor import QuadrotorState, QuadrotorParams
from src.controllers.mpc import MPCSolver, MPCConfig
from src.controllers.geometric import GeometricController
from src.consensus.protocol import ConsensusProtocol
from src.formation.planner import FormationPlanner, FormationConfig


@dataclass
class UAV:
    id: int
    state: QuadrotorState
    mpc: MPCSolver
    controller: GeometricController


class SimulationEnvironment:
    """Simulation environment for multi-UAV formation control."""
    
    def __init__(self, num_uavs: int, formation_type: str = 'grid'):
        self.num_uavs = num_uavs
        self.params = QuadrotorParams()
        
        self.uavs: List[UAV] = []
        for i in range(num_uavs):
            state = QuadrotorState(
                position=np.array([float(i % 3), float(i // 3), 1.0]),
                velocity=np.zeros(3),
                attitude=np.array([1.0, 0.0, 0.0, 0.0]),
                angular_velocity=np.zeros(3)
            )
            self.uavs.append(UAV(
                id=i,
                state=state,
                mpc=MPCSolver(MPCConfig(horizon=10, dt=0.1), num_uavs=num_uavs),
                controller=GeometricController()
            ))
        
        self.consensus = ConsensusProtocol(num_uavs, 'ring')
        self.planner = FormationPlanner(FormationConfig(
            num_uavs=num_uavs, 
            spacing=1.0, 
            formation_type=formation_type
        ))
        
        self.time = 0.0
        self.dt = 0.05
        self.max_time = 50.0
    
    def step(self, target_center: np.ndarray) -> Dict:
        """One simulation step."""
        formation_offsets = self.planner.compute_offsets(target_center)
        
        for uav in self.uavs:
            neighbor_ids = self.consensus.get_neighbors(uav.id)
            neighbor_positions = [self.uavs[n].state.position for n in neighbor_ids]
            
            consensus_pos = self.consensus.update(uav.state.position, neighbor_positions)
            
            target_pos = self.planner.get_target_position(uav.id, target_center, formation_offsets)
            
            ref_traj = np.tile(target_pos, (10, 1))
            
            current_state = np.concatenate([uav.state.position, uav.state.velocity])
            
            kp = 0.3
            kd = 0.5
            
            pos_error = target_pos - uav.state.position
            
            desired_velocity = kp * pos_error
            
            vel_error = desired_velocity - uav.state.velocity
            acceleration = kd * vel_error
            
            new_velocity = uav.state.velocity + acceleration * self.dt
            
            new_velocity = np.clip(new_velocity, -3.0, 3.0)
            
            new_position = uav.state.position + new_velocity * self.dt
            
            uav.state.position = new_position
            uav.state.velocity = new_velocity
        
        self.time += self.dt
        
        errors = [np.linalg.norm(u.state.position - target_center - formation_offsets[i]) 
                  for i, u in enumerate(self.uavs)]
        
        return {
            'time': self.time,
            'positions': [u.state.position.copy() for u in self.uavs],
            'mean_error': np.mean(errors),
            'max_error': np.max(errors)
        }
    
    def _update_state(self, state: QuadrotorState, u: np.ndarray, dt: float) -> QuadrotorState:
        return self._update_state_annotated(state, u, dt)
    
    def _update_state_annotated(self, state: QuadrotorState, u: np.ndarray, dt: float) -> QuadrotorState:
        thrust = u[0]
        roll = u[1]
        pitch = u[2]
        
        thrust_vec = np.array([0, 0, thrust])
        
        roll_rad = roll
        pitch_rad = pitch
        
        R = np.array([
            [np.cos(roll_rad), np.sin(roll_rad) * np.sin(pitch_rad), np.sin(roll_rad) * np.cos(pitch_rad)],
            [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
            [-np.sin(roll_rad), np.cos(roll_rad) * np.sin(pitch_rad), np.cos(roll_rad) * np.cos(pitch_rad)]
        ])
        
        body_thrust = R @ thrust_vec
        
        gravity = np.array([0, 0, -self.params.gravity])
        acc = gravity + body_thrust / self.params.mass
        
        new_velocity = state.velocity + acc * dt
        new_position = state.position + new_velocity * dt
        
        return QuadrotorState(
            position=new_position,
            velocity=new_velocity,
            attitude=state.attitude.copy(),
            angular_velocity=np.zeros(3)
        )
    
    def _update_state_simple(self, state: QuadrotorState, acc: np.ndarray, dt: float) -> QuadrotorState:
        new_velocity = state.velocity + acc * dt
        new_position = state.position + new_velocity * dt
        
        return QuadrotorState(
            position=new_position,
            velocity=new_velocity,
            attitude=state.attitude.copy(),
            angular_velocity=np.zeros(3)
        )
    
    def run(self, target_center: np.ndarray = None) -> Dict:
        """Run full simulation."""
        if target_center is None:
            target_center = np.array([5.0, 0.0, 2.0])
        
        results = []
        while self.time < self.max_time:
            result = self.step(target_center)
            results.append(result)
            if result['mean_error'] < 0.1:
                break
        
        return {
            'success': result['mean_error'] < 0.3,
            'final_error': result['mean_error'],
            'steps': len(results),
            'positions': result['positions']
        }
    
    def simulate_trajectory(self, trajectory: List[np.ndarray]) -> List[Dict]:
        """Simulate following a trajectory of target positions."""
        results = []
        for target in trajectory:
            result = self.step(target)
            results.append(result)
        return results