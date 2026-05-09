import numpy as np
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.consensus.protocol import ConsensusProtocol

def test_consensus_init():
    protocol = ConsensusProtocol(num_uavs=4, topology='ring')
    assert protocol.num_uavs == 4
    assert protocol.topology == 'ring'

def test_consensus_ring_topology():
    protocol = ConsensusProtocol(num_uavs=4, topology='ring')
    neighbors = protocol.get_neighbors(0)
    assert 3 in neighbors
    assert 1 in neighbors

def test_consensus_update():
    protocol = ConsensusProtocol(num_uavs=3, topology='ring')
    my_state = np.array([0.0, 0.0, 1.0])
    neighbor_states = [np.array([1.0, 0.0, 1.0]), np.array([0.0, 1.0, 1.0])]
    consensus_state = protocol.update(my_state, neighbor_states)
    assert consensus_state.shape == (3,)

if __name__ == '__main__':
    test_consensus_init()
    test_consensus_ring_topology()
    test_consensus_update()
    print("All consensus tests passed!")