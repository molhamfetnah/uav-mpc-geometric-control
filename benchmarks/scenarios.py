import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from typing import Dict, Callable, List
from src.simulation.environment import SimulationEnvironment
from src.formation.planner import FormationConfig


SCENARIOS = {
    'S1': {
        'name': 'Formation Hold',
        'target': np.array([5.0, 0.0, 2.0]),
        'formation': 'grid',
        'expected_success': True
    },
    'S2': {
        'name': 'Formation Translation',
        'target': np.array([10.0, 5.0, 3.0]),
        'formation': 'line',
        'expected_success': True
    },
    'S3': {
        'name': 'Formation Rotation',
        'target': np.array([5.0, 0.0, 2.0]),
        'formation': 'circle',
        'expected_success': True
    },
    'S4': {
        'name': 'Dynamic Target Tracking',
        'target': lambda t: np.array([5 + 0.1*t, np.sin(0.1*t), 2]),
        'formation': 'grid',
        'expected_success': True
    },
    'S5': {
        'name': 'Obstacle Avoidance',
        'target': np.array([10.0, 0.0, 2.0]),
        'obstacles': [np.array([7.0, 0.0, 2.0])],
        'formation': 'grid',
        'expected_success': True
    },
    'S6': {
        'name': 'Communication Loss',
        'target': np.array([5.0, 0.0, 2.0]),
        'packet_loss': 0.5,
        'formation': 'grid',
        'expected_success': True
    },
    'S7': {
        'name': 'Variable Swarm (8 UAVs)',
        'target': np.array([5.0, 0.0, 2.0]),
        'num_uavs': 8,
        'formation': 'grid',
        'expected_success': True
    }
}


def run_benchmark(scenario_id: str, verbose: bool = True) -> Dict:
    """Run a single benchmark scenario."""
    if scenario_id not in SCENARIOS:
        return {'error': f'Unknown scenario {scenario_id}'}
    
    config = SCENARIOS[scenario_id]
    num_uavs = config.get('num_uavs', 4)
    formation = config.get('formation', 'grid')
    target = config.get('target', np.array([5.0, 0.0, 2.0]))
    
    env = SimulationEnvironment(num_uavs, formation)
    
    if callable(target):
        results = []
        for step in range(50):
            t = step * 0.1
            current_target = target(t)
            result = env.step(current_target)
            results.append(result)
        final_result = results[-1]
        final_error = final_result.get('mean_error', float('inf'))
    else:
        result = env.run(target)
        results = [result]
        final_error = result.get('final_error', float('inf'))
    
    success = final_error < 5.0
    
    if verbose:
        status = 'PASS' if success else 'FAIL'
        print(f"{scenario_id}: {config['name']} - {status} (error: {final_error:.3f})")
    
    return {
        'scenario': scenario_id,
        'name': config['name'],
        'success': success,
        'final_error': final_error,
        'expected_success': config.get('expected_success', True)
    }


def run_all_benchmarks(verbose: bool = True) -> Dict:
    """Run all benchmark scenarios."""
    results = []
    for sid in SCENARIOS:
        result = run_benchmark(sid, verbose)
        results.append(result)
    
    success_count = sum(1 for r in results if r.get('success', False))
    total = len(results)
    success_rate = success_count / total * 100
    
    if verbose:
        print(f"\nOverall: {success_count}/{total} ({success_rate:.1f}%) success rate")
    
    return {
        'results': results,
        'success_count': success_count,
        'total': total,
        'success_rate': success_rate
    }


if __name__ == '__main__':
    run_all_benchmarks()