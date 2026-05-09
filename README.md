# UAV MPC Geometric Control

## Project Overview

Distributed Model Predictive Control (DMPC) for multi-UAV quadrotor formation with consensus-based coordination and geometric SO(3) control.

## Key Features

- **Distributed MPC**: Each UAV runs local optimization
- **Consensus Protocol**: Ring topology for neighbor coordination
- **Geometric Controller**: SO(3) attitude control with quaternion
- **Formation Planner**: Grid, line, circle, wedge formations
- **Simulation Environment**: Full 6-DOF quadrotor simulation
- **Benchmark Suite**: 7 scenarios for evaluation

## Architecture

```
Multi-UAV Formation
├── Quadrotor Model (6-DOF dynamics)
├── Local MPC Solver
├── Geometric Controller (SO3)
├── Consensus Protocol
└── Formation Planner
```

## Installation

```bash
pip install numpy scipy matplotlib
```

## Quick Start

```python
import numpy as np
from src.simulation.environment import SimulationEnvironment

env = SimulationEnvironment(num_uavs=4, formation_type='grid')
result = env.run(target=np.array([5.0, 0.0, 2.0]))
print(f"Success: {result['success']}, Error: {result['final_error']:.3f}")
```

## Run Benchmarks

```bash
python benchmarks/scenarios.py
```

## Status

- **Paper**: In preparation for IEEE T-RO / RA-L
- **Implementation**: Core framework complete
- **Next**: Control tuning, hardware validation

## License

MIT License