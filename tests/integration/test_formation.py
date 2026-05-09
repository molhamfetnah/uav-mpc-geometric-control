import numpy as np
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.simulation.environment import SimulationEnvironment, UAV

def test_simulation_init():
    env = SimulationEnvironment(num_uavs=4)
    assert env.num_uavs == 4

def test_simulation_step():
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([5.0, 0.0, 2.0])
    result = env.step(target)
    assert 'time' in result
    assert 'positions' in result

def test_simulation_run():
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    assert 'success' in result
    assert 'final_error' in result
    assert 'steps' in result

if __name__ == '__main__':
    test_simulation_init()
    test_simulation_step()
    test_simulation_run()
    print("All integration tests passed!")