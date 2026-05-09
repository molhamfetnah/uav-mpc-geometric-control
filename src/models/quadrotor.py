import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class QuadrotorState:
    position: np.ndarray = None
    velocity: np.ndarray = None
    attitude: np.ndarray = None
    angular_velocity: np.ndarray = None
    
    def __post_init__(self):
        if self.position is None:
            self.position = np.zeros(3)
        if self.velocity is None:
            self.velocity = np.zeros(3)
        if self.attitude is None:
            self.attitude = np.array([1.0, 0.0, 0.0, 0.0])
        if self.angular_velocity is None:
            self.angular_velocity = np.zeros(3)


@dataclass
class QuadrotorParams:
    mass: float = 1.0
    inertia: np.ndarray = None
    gravity: float = 9.81
    thrust_coefficient: float = 1.0
    drag_coefficient: float = 0.1
    
    def __post_init__(self):
        if self.inertia is None:
            self.inertia = np.diag([0.01, 0.01, 0.02])


class QuadrotorModel:
    def __init__(self, params: QuadrotorParams):
        self.params = params
    
    def dynamics(self, state: QuadrotorState, u: np.ndarray, dt: float = 0.01) -> QuadrotorState:
        thrust = u[0]
        tau = u[1:4] if len(u) > 1 else np.zeros(3)
        
        R = self._quaternion_to_rotation(state.attitude)
        thrust_dir = R @ np.array([0, 0, 1])
        
        acc = np.array([0, 0, -self.params.gravity]) + thrust * thrust_dir / self.params.mass
        
        if self.params.inertia is not None:
            angular_acc = np.linalg.inv(self.params.inertia) @ (tau - np.cross(state.angular_velocity, self.params.inertia @ state.angular_velocity))
        else:
            angular_acc = np.zeros(3)
        
        new_position = state.position + state.velocity * dt
        new_velocity = state.velocity + acc * dt
        
        w, x, y, z = state.attitude
        q_dot = 0.5 * np.array([
            [-x, -y, -z],
            [w, -z, y],
            [z, w, -x],
            [-y, x, w]
        ]) @ state.angular_velocity
        new_attitude = state.attitude + q_dot * dt
        new_attitude = new_attitude / np.linalg.norm(new_attitude)
        
        new_angular_velocity = state.angular_velocity + angular_acc * dt
        
        return QuadrotorState(
            position=new_position,
            velocity=new_velocity,
            attitude=new_attitude,
            angular_velocity=new_angular_velocity
        )
    
    def _quaternion_to_rotation(self, q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
    
    def forward_kinematics(self, state: QuadrotorState) -> np.ndarray:
        return state.position.copy()
    
    def linearized_dynamics(self, state: QuadrotorState) -> tuple:
        n = 6
        m = 4
        A = np.zeros((n, n))
        B = np.zeros((n, m))
        
        A[:3, 3:] = np.eye(3)
        
        return A, B