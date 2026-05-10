# UAV MPC Geometric Control

## Distributed Model Predictive Control for Multi-UAV Formation

A comprehensive implementation of distributed MPC for multi-UAV quadrotor formation control with consensus-based coordination and geometric SO(3) attitude control.

## Features

- **Distributed Architecture**: Each UAV runs local MPC without central coordinator
- **Consensus Protocol**: Ring/mesh/star topology for neighbor coordination
- **Geometric Control**: SO(3) attitude controller using quaternion representation
- **Formation Planning**: Grid, line, circle, wedge formations
- **Comprehensive Benchmarks**: 7 scenarios with 100% pass rate

## Architecture

```
Multi-UAV Formation System
├── Quadrotor Dynamics (6-DOF)
├── Local MPC Controller
├── Geometric SO3 Controller
├── Consensus Protocol
└── Formation Planner
```

## Installation

```bash
git clone https://github.com/molhamfetnah/uav-mpc-geometric-control.git
cd uav-mpc-geometric-control
pip install -r requirements.txt
```

## Quick Start

```python
import numpy as np
from src.simulation.environment import SimulationEnvironment

# Create environment with 4 UAVs in grid formation
env = SimulationEnvironment(num_uavs=4, formation_type='grid')

# Run simulation to target position
result = env.run(target=np.array([5.0, 0.0, 2.0]))

print(f"Success: {result['success']}")
print(f"Final Error: {result['final_error']:.3f} m")
```

## Run Benchmarks

```bash
cd uav-mpc-geometric-control
python benchmarks/scenarios.py
```

### Benchmark Results

| Scenario | Description | Error | Status |
|----------|-------------|-------|--------|
| S1 | Formation Hold | 0.092m | ✅ PASS |
| S2 | Formation Translation | 0.100m | ✅ PASS |
| S3 | Formation Rotation | 0.099m | ✅ PASS |
| S4 | Dynamic Target Tracking | 3.866m | ✅ PASS |
| S5 | Obstacle Avoidance | 0.092m | ✅ PASS |
| S6 | Communication Loss | 0.092m | ✅ PASS |
| S7 | Variable Swarm (8 UAVs) | 0.096m | ✅ PASS |

**Overall: 7/7 (100.0%) success rate**

## Project Structure

```
uav-mpc-geometric-control/
├── src/
│   ├── models/
│   │   └── quadrotor.py          # Quadrotor dynamics
│   ├── controllers/
│   │   ├── mpc.py                # MPC solver
│   │   └── geometric.py         # SO3 controller
│   ├── consensus/
│   │   └── protocol.py           # Consensus logic
│   ├── formation/
│   │   └── planner.py            # Formation shapes
│   └── simulation/
│       └── environment.py        # Main simulation
├── tests/
│   ├── unit/                     # Unit tests
│   └── integration/              # Integration tests
├── benchmarks/
│   └── scenarios.py              # 7 benchmark scenarios
├── paper/
│   ├── journal/
│   │   └── SUBMISSION_GUIDE.md   # Submission guide
│   └── manuscript.md             # Paper draft
├── SPEC.md                       # Full specification
├── README.md                     # This file
└── requirements.txt              # Dependencies
```

## Running Tests

```bash
# Run unit tests
python -m pytest tests/unit/ -v

# Run integration tests
python -m pytest tests/integration/ -v

# Run benchmarks
python benchmarks/scenarios.py
```

## Paper

This implementation supports a paper submission to IEEE Transactions on Robotics (T-RO) or IEEE Robotics and Automation Letters (RA-L).

**Paper Title**: Distributed Model Predictive Control for Multi-UAV Formation with Consensus

**Key Contributions**:
1. Local MPC solver with formation constraints
2. Consensus protocol for decentralized coordination
3. Geometric SO(3) attitude controller
4. 100% success rate across 7 benchmark scenarios

## Stress Tests

Run comprehensive edge case tests:
```bash
python tests/stress/test_edge_cases.py
```

**Results: 95.2% pass (20/21 tests)**

| Category | Tests | Pass |
|----------|-------|------|
| Extreme Positions | 5 | 100% |
| Swarm Scaling (2-15 UAVs) | 3 | 100% |
| Formation Types | 4 | 100% |
| Initial States | 3 | 100% |
| Consensus Protocols | 3 | 100% |
| Combined Stress | 1 | 100% |
| Dynamic Behavior | 2 | 50%* |

*Note: Rapid trajectory change exceeds threshold - see STRESS_TEST_REPORT.md

Full report: `tests/stress/STRESS_TEST_REPORT.md`

## Status

- **Implementation**: ✅ Complete
- **Benchmarks**: ✅ 100% (7/7)
- **Stress Tests**: ✅ 95.2% (20/21)
- **Paper**: 📝 Draft ready
- **Submission**: ⏳ Pending (IEEE T-RO/RA-L)

## License

MIT License - See LICENSE file

## Author

Mulham Fetna
- University of Aleppo
- Department of Mechatronics Engineering
- Email: mulham.fetna@alepuniv.edu.sy

---

*Last Updated: May 9, 2026*