import numpy as np
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.controllers.geometric import GeometricController
from src.models.quadrotor import QuadrotorState

def test_geometric_controller_init():
    controller = GeometricController()
    assert controller is not None
    assert hasattr(controller, 'control')

def test_geometric_controller_output_shape():
    controller = GeometricController()
    state = QuadrotorState(
        position=np.array([0.0, 0.0, 1.0]),
        velocity=np.zeros(3),
        attitude=np.array([1.0, 0.0, 0.0, 0.0]),
        angular_velocity=np.zeros(3)
    )
    desired_pos = np.array([1.0, 0.0, 1.0])
    desired_vel = np.zeros(3)
    desired_acc = np.zeros(3)
    u = controller.control(state, desired_pos, desired_vel, desired_acc)
    assert u.shape == (4,)

if __name__ == '__main__':
    test_geometric_controller_init()
    test_geometric_controller_output_shape()
    print("All geometric controller tests passed!")