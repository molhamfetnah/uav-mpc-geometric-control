import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.models.quadrotor import QuadrotorState, QuadrotorParams, QuadrotorModel

def test_quadrotor_initialization():
    params = QuadrotorParams()
    state = QuadrotorState()
    model = QuadrotorModel(params)
    assert state.position.shape == (3,)
    assert state.velocity.shape == (3,)

def test_quadrotor_params_default():
    params = QuadrotorParams()
    assert params.mass == 1.0
    assert params.gravity == 9.81
    assert np.allclose(params.inertia, np.diag([0.01, 0.01, 0.02]))

def test_quadrotor_dynamics():
    params = QuadrotorParams()
    model = QuadrotorModel(params)
    state = QuadrotorState(
        position=np.array([0.0, 0.0, 1.0]),
        velocity=np.zeros(3),
        attitude=np.array([1.0, 0.0, 0.0, 0.0]),
        angular_velocity=np.zeros(3)
    )
    u = np.array([9.81, 0.0, 0.0, 0.0])  # Hover thrust
    new_state = model.dynamics(state, u)
    assert new_state is not None
    assert new_state.position.shape == (3,)

if __name__ == '__main__':
    test_quadrotor_initialization()
    test_quadrotor_params_default()
    test_quadrotor_dynamics()
    print("All tests passed!")