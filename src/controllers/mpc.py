import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class MPCConfig:
    horizon: int = 15
    dt: float = 0.05
    max_thrust: float = 20.0
    max_torque: float = 1.0
    tracking_weight: float = 1.0
    control_weight: float = 0.1
    formation_weight: float = 0.5


class MPCSolver:
    """Model Predictive Controller for quadrotor formation."""
    
    def __init__(self, config: Optional[MPCConfig] = None, num_uavs: int = 1, horizon: int = None, dt: float = None):
        if config is None:
            config = MPCConfig(horizon=horizon or 15, dt=dt or 0.05)
        self.config = config
        self.horizon = self.config.horizon
        self.dt = self.config.dt
        self.num_uavs = num_uavs
        self.n_states = 6
        self.n_inputs = 4
    
    def solve(self, current_state: np.ndarray, 
              reference_traj: np.ndarray,
              formation_offsets: np.ndarray,
              neighbor_states: list = None) -> Tuple[np.ndarray, bool]:
        """
        Solve MPC optimization problem using gradient descent.
        
        Args:
            current_state: [x, y, z, vx, vy, vz]
            reference_traj: (horizon, 3) array of reference positions
            formation_offsets: (N, 3) relative positions in formation
            neighbor_states: list of neighbor UAV states
        
        Returns:
            optimal_input: (4,) control input
            success: bool
        """
        U = np.zeros((self.horizon, self.n_inputs))
        
        for _ in range(50):
            X = self._simulate_trajectory(current_state, U)
            grad = self._compute_gradient(X, U, reference_traj, formation_offsets)
            U -= 0.01 * grad
            U = np.clip(U, -self.config.max_thrust, self.config.max_thrust)
        
        return U[0], True
    
    def _simulate_trajectory(self, x0: np.ndarray, U: np.ndarray) -> np.ndarray:
        X = np.zeros((self.horizon + 1, self.n_states))
        X[0] = x0.copy()
        
        for t in range(self.horizon):
            x = X[t]
            u = U[t]
            
            acc = np.array([0, 0, -9.81]) + u[0] * np.array([0, 0, 1]) / 1.0
            X[t+1, :3] = x[:3] + x[3:] * self.dt
            X[t+1, 3:] = x[3:] + acc * self.dt
        
        return X
    
    def _compute_gradient(self, X: np.ndarray, U: np.ndarray, 
                          ref_traj: np.ndarray, formation_offsets: np.ndarray) -> np.ndarray:
        grad = np.zeros_like(U)
        
        for t in range(self.horizon):
            pos = X[t, :3]
            ref = ref_traj[t] if t < len(ref_traj) else ref_traj[-1]
            
            grad[t, 0] = self.config.tracking_weight * 2 * (pos[2] - ref[2]) * self.dt
            
            for i in range(3):
                grad[t, 0] += self.config.tracking_weight * 2 * (pos[i] - ref[i]) * self.dt
                grad[t] += self.config.control_weight * 2 * U[t] * self.dt
        
        return grad
    
    def solve_analytical(self, current_state: np.ndarray,
                        reference_traj: np.ndarray) -> Tuple[np.ndarray, bool]:
        """
        Simple analytical solution for testing.
        """
        if len(reference_traj) == 0:
            return np.zeros(4), False
        
        target = reference_traj[0]
        error = target - current_state[:3]
        
        Kp = 5.0
        Kv = 2.0
        
        desired_acc = Kp * error + Kv * (-current_state[3:])
        desired_acc[2] += 9.81
        
        thrust = np.linalg.norm(desired_acc) * 1.0
        if thrust < 1e-6:
            thrust = 9.81
        
        return np.array([thrust, 0.0, 0.0, 0.0]), True