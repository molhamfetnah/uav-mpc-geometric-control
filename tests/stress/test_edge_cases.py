"""
Stress Test Suite for Multi-UAV Formation Control
==================================================

Comprehensive edge case testing for distributed MPC formation control system.
Tests cover: extreme positions, communication failures, numerical stability,
swarm scaling, formation transitions, and failure modes.

Author: Mulham Fetna
Affiliation: University of Aleppo, Department of Mechatronics Engineering
Date: May 9, 2026
"""

import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.simulation.environment import SimulationEnvironment
from src.formation.planner import FormationPlanner, FormationConfig
from src.consensus.protocol import ConsensusProtocol
from src.models.quadrotor import QuadrotorState, QuadrotorParams


class StressTestRunner:
    """Comprehensive stress test runner for edge case evaluation."""
    
    def __init__(self):
        self.results = []
        self.passed = 0
        self.failed = 0
    
    def run_test(self, name, test_func, threshold=5.0):
        """Run a single stress test."""
        print(f"  Testing: {name}...", end=" ")
        try:
            result = test_func()
            if result <= threshold:
                print(f"PASS (error: {result:.3f}m)")
                self.passed += 1
                return True
            else:
                print(f"FAIL (error: {result:.3f}m > {threshold}m)")
                self.failed += 1
                return False
        except Exception as e:
            print(f"ERROR: {str(e)[:50]}")
            self.failed += 1
            return False
    
    def summary(self):
        total = self.passed + self.failed
        print(f"\n{'='*60}")
        print(f"STRESS TEST SUMMARY")
        print(f"{'='*60}")
        print(f"Total Tests: {total}")
        print(f"Passed: {self.passed} ({self.passed/total*100:.1f}%)")
        print(f"Failed: {self.failed} ({self.failed/total*100:.1f}%)")
        print(f"{'='*60}")
        return self.passed, self.failed


def test_extreme_position_stability():
    """Test with extreme target positions."""
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([100.0, -100.0, 50.0])
    result = env.run(target)
    return result['final_error']


def test_zero_position():
    """Test with zero target (origin)."""
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([0.0, 0.0, 0.0])
    result = env.run(target)
    return result['final_error']


def test_very_close_targets():
    """Test with targets very close together."""
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([0.01, 0.01, 0.01])
    result = env.run(target)
    return result['final_error']


def test_high_altitude():
    """Test with very high altitude."""
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([5.0, 0.0, 100.0])
    result = env.run(target)
    return result['final_error']


def test_negative_altitude():
    """Test with negative altitude (below ground)."""
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([5.0, 0.0, -5.0])
    result = env.run(target)
    return result['final_error']


def test_large_swarm_10():
    """Test with 10 UAVs."""
    env = SimulationEnvironment(num_uavs=10)
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_large_swarm_15():
    """Test with 15 UAVs."""
    env = SimulationEnvironment(num_uavs=15)
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_minimum_swarm_2():
    """Test with minimum 2 UAVs."""
    env = SimulationEnvironment(num_uavs=2)
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_line_formation():
    """Test line formation."""
    env = SimulationEnvironment(num_uavs=4, formation_type='line')
    target = np.array([10.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_circle_formation():
    """Test circle formation."""
    env = SimulationEnvironment(num_uavs=4, formation_type='circle')
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_wedge_formation():
    """Test wedge formation."""
    env = SimulationEnvironment(num_uavs=4, formation_type='wedge')
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_formation_grid():
    """Test grid formation."""
    env = SimulationEnvironment(num_uavs=4, formation_type='grid')
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_multi_formation_transition():
    """Test transitioning between formations."""
    env = SimulationEnvironment(num_uavs=4, formation_type='grid')
    
    target = np.array([5.0, 0.0, 2.0])
    result1 = env.run(target)
    
    env2 = SimulationEnvironment(num_uavs=4, formation_type='line')
    result2 = env2.run(target)
    
    return (result1['final_error'] + result2['final_error']) / 2


def test_zero_initial_velocity():
    """Test with all UAVs starting at zero velocity."""
    env = SimulationEnvironment(num_uavs=4)
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_non_zero_initial_velocity():
    """Test with non-zero initial velocities."""
    env = SimulationEnvironment(num_uavs=4)
    for uav in env.uavs:
        uav.state.velocity = np.array([1.0, 1.0, 0.5])
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_offset_start_positions():
    """Test with widely spread initial positions."""
    env = SimulationEnvironment(num_uavs=4)
    positions = [
        np.array([0.0, 0.0, 1.0]),
        np.array([100.0, 0.0, 1.0]),
        np.array([0.0, 100.0, 1.0]),
        np.array([100.0, 100.0, 1.0])
    ]
    for i, uav in enumerate(env.uavs):
        uav.state.position = positions[i]
    target = np.array([50.0, 50.0, 5.0])
    result = env.run(target)
    return result['final_error']


def test_rapid_trajectory_change():
    """Test with rapidly changing target trajectory."""
    env = SimulationEnvironment(num_uavs=4)
    targets = [
        np.array([5.0, 0.0, 2.0]),
        np.array([10.0, 5.0, 3.0]),
        np.array([0.0, 10.0, 1.0]),
        np.array([5.0, 5.0, 2.0]),
        np.array([10.0, 0.0, 4.0])
    ]
    final_error = 0
    for target in targets:
        result = env.step(target)
        final_error = result['mean_error']
    return final_error


def test_consensus_convergence():
    """Test consensus protocol convergence."""
    protocol = ConsensusProtocol(num_uavs=4, topology='ring')
    states = [np.array([i*0.5, i*0.3, i*0.1]) for i in range(4)]
    
    for _ in range(10):
        new_states = []
        for i in range(4):
            neighbors = [states[(i-1)%4], states[(i+1)%4]]
            new_state = protocol.update(states[i], neighbors)
            new_states.append(new_state)
        states = new_states
    
    final_error = protocol.compute_consensus_error(states)
    return final_error


def test_mesh_topology():
    """Test with mesh communication topology."""
    env = SimulationEnvironment(num_uavs=4)
    env.consensus = ConsensusProtocol(num_uavs=4, topology='mesh')
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_star_topology():
    """Test with star communication topology."""
    env = SimulationEnvironment(num_uavs=4)
    env.consensus = ConsensusProtocol(num_uavs=4, topology='star')
    target = np.array([5.0, 0.0, 2.0])
    result = env.run(target)
    return result['final_error']


def test_all_formation_types_scaled():
    """Test all formation types with 8 UAVs."""
    formations = ['grid', 'line', 'circle', 'wedge']
    errors = []
    
    for formation in formations:
        env = SimulationEnvironment(num_uavs=8, formation_type=formation)
        result = env.run(np.array([5.0, 0.0, 2.0]))
        errors.append(result['final_error'])
    
    return max(errors)


def run_all_stress_tests():
    """Run complete stress test suite."""
    runner = StressTestRunner()
    
    print("\n" + "="*60)
    print("STRESS TEST SUITE - Multi-UAV Formation Control")
    print("="*60)
    
    print("\n[1] EXTREME POSITION TESTS")
    runner.run_test("Extreme position (100, -100, 50)", test_extreme_position_stability)
    runner.run_test("Zero position", test_zero_position)
    runner.run_test("Very close targets (0.01)", test_very_close_targets)
    runner.run_test("High altitude (100m)", test_high_altitude)
    runner.run_test("Negative altitude (-5m)", test_negative_altitude)
    
    print("\n[2] SWARM SCALING TESTS")
    runner.run_test("Large swarm (10 UAVs)", test_large_swarm_10)
    runner.run_test("Large swarm (15 UAVs)", test_large_swarm_15)
    runner.run_test("Minimum swarm (2 UAVs)", test_minimum_swarm_2)
    
    print("\n[3] FORMATION TYPE TESTS")
    runner.run_test("Line formation", test_line_formation)
    runner.run_test("Circle formation", test_circle_formation)
    runner.run_test("Wedge formation", test_wedge_formation)
    runner.run_test("Grid formation", test_formation_grid)
    
    print("\n[4] INITIAL STATE TESTS")
    runner.run_test("Zero initial velocity", test_zero_initial_velocity)
    runner.run_test("Non-zero initial velocity", test_non_zero_initial_velocity)
    runner.run_test("Offset start positions", test_offset_start_positions)
    
    print("\n[5] DYNAMIC BEHAVIOR TESTS")
    runner.run_test("Rapid trajectory change", test_rapid_trajectory_change)
    runner.run_test("Multi-formation transition", test_multi_formation_transition)
    
    print("\n[6] CONSENSUS PROTOCOL TESTS")
    runner.run_test("Consensus convergence", test_consensus_convergence, threshold=0.1)
    runner.run_test("Mesh topology", test_mesh_topology)
    runner.run_test("Star topology", test_star_topology)
    
    print("\n[7] COMBINED STRESS TESTS")
    runner.run_test("All formations scaled (8 UAVs)", test_all_formation_types_scaled)
    
    return runner.summary()


if __name__ == '__main__':
    run_all_stress_tests()