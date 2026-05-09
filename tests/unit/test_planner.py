import numpy as np
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.formation.planner import FormationPlanner, FormationConfig

def test_formation_planner_init():
    config = FormationConfig(num_uavs=4, formation_type='grid')
    planner = FormationPlanner(config)
    assert planner.formation_type == 'grid'
    assert planner.config.num_uavs == 4

def test_formation_grid():
    config = FormationConfig(num_uavs=4, spacing=1.0, formation_type='grid')
    planner = FormationPlanner(config)
    offsets = planner.compute_offsets(np.array([0.0, 0.0, 0.0]))
    assert offsets.shape == (4, 3)

def test_formation_line():
    config = FormationConfig(num_uavs=4, spacing=2.0, formation_type='line')
    planner = FormationPlanner(config)
    offsets = planner.compute_offsets(np.array([0.0, 0.0, 0.0]))
    assert offsets.shape == (4, 3)
    assert offsets[0, 0] < offsets[-1, 0]

def test_formation_circle():
    config = FormationConfig(num_uavs=4, spacing=1.0, formation_type='circle')
    planner = FormationPlanner(config)
    offsets = planner.compute_offsets(np.array([0.0, 0.0, 0.0]))
    assert offsets.shape == (4, 3)

def test_get_target_position():
    config = FormationConfig(num_uavs=4, formation_type='grid')
    planner = FormationPlanner(config)
    center = np.array([5.0, 0.0, 2.0])
    offsets = planner.compute_offsets(center)
    target = planner.get_target_position(0, center, offsets)
    assert target.shape == (3,)

if __name__ == '__main__':
    test_formation_planner_init()
    test_formation_grid()
    test_formation_line()
    test_formation_circle()
    test_get_target_position()
    print("All formation planner tests passed!")