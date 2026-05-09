import numpy as np
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.controllers.mpc import MPCSolver, MPCConfig

def test_mpc_solver_init():
    solver = MPCSolver(horizon=10, dt=0.05)
    assert solver.horizon == 10
    assert solver.dt == 0.05

def test_mpc_solver_config():
    config = MPCConfig(horizon=15, dt=0.1, max_thrust=20.0)
    solver = MPCSolver(config)
    assert solver.config.horizon == 15

def test_mpc_solver_solve():
    solver = MPCSolver(MPCConfig(horizon=5, dt=0.1))
    current_state = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    ref_traj = np.tile(np.array([1.0, 0.0, 1.0]), (5, 1))
    formation_offsets = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    
    u, success = solver.solve(current_state, ref_traj, formation_offsets)
    assert u.shape == (4,)
    assert isinstance(success, bool)

if __name__ == '__main__':
    test_mpc_solver_init()
    test_mpc_solver_config()
    test_mpc_solver_solve()
    print("All MPC tests passed!")