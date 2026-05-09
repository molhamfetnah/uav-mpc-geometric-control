import numpy as np
from dataclasses import dataclass
from typing import Optional
from src.models.quadrotor import QuadrotorState, QuadrotorParams


@dataclass
class GeometricController:
    """SO(3) geometric attitude controller for quadrotors."""
    k_r: float = 5.0
    k_w: float = 1.0
    k_thrust: float = 1.0
    params: Optional[QuadrotorParams] = None
    
    def __post_init__(self):
        if self.params is None:
            self.params = QuadrotorParams()
    
    def control(self, state: QuadrotorState, 
                desired_pos: np.ndarray, 
                desired_vel: np.ndarray, 
                desired_acc: np.ndarray) -> np.ndarray:
        """
        Compute motor commands given current state and desired trajectory.
        
        Returns: [thrust, tau_x, tau_y, tau_z]
        """
        e_pos = desired_pos - state.position
        e_vel = desired_vel - state.velocity
        
        desired_thrust = desired_acc + self.params.gravity * np.array([0, 0, 1]) + self.k_r * e_pos + 0.5 * e_vel
        
        thrust_magnitude = np.linalg.norm(desired_thrust)
        if thrust_magnitude < 1e-6:
            thrust_magnitude = self.params.gravity * self.params.mass
        
        R = self._quaternion_to_rotation(state.attitude)
        z_body = R @ np.array([0, 0, 1])
        
        e_R = self._compute_rotation_error(R, desired_thrust, thrust_magnitude, z_body)
        e_omega = state.angular_velocity
        
        tau = -self.k_w * e_omega + np.cross(state.angular_velocity, self.params.inertia @ state.angular_velocity)
        
        return np.array([thrust_magnitude, tau[0], tau[1], tau[2]])
    
    def _quaternion_to_rotation(self, q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
    
    def _compute_rotation_error(self, R: np.ndarray, desired_thrust: np.ndarray, 
                                 thrust_magnitude: float, z_body: np.ndarray) -> np.ndarray:
        desired_R_desired = np.eye(3)
        if thrust_magnitude > 1e-6:
            desired_z = desired_thrust / thrust_magnitude
            x_desired = np.cross(np.array([0, 1, 0]), desired_z)
            if np.linalg.norm(x_desired) < 1e-6:
                x_desired = np.cross(np.array([1, 0, 0]), desired_z)
            x_desired = x_desired / np.linalg.norm(x_desired)
            y_desired = np.cross(desired_z, x_desired)
            desired_R_desired = np.column_stack([x_desired, y_desired, desired_z])
        
        R_error = desired_R_desired.T @ R - R.T @ desired_R_desired
        e_R = np.array([R_error[2, 1], R_error[0, 2], R_error[1, 0]])
        
        return e_R