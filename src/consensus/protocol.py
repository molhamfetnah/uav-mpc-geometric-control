import numpy as np
from typing import List, Optional, Dict


class ConsensusProtocol:
    """Consensus protocol for multi-UAV formation."""
    
    def __init__(self, num_uavs: int, topology: str = 'ring'):
        self.num_uavs = num_uavs
        self.topology = topology
        self.adjacency = self._build_adjacency()
        
    def _build_adjacency(self) -> Dict[int, List[int]]:
        """Build communication graph adjacency list."""
        if self.topology == 'ring':
            return {i: [(i-1) % self.num_uavs, (i+1) % self.num_uavs] for i in range(self.num_uavs)}
        elif self.topology == 'mesh':
            return {i: [j for j in range(self.num_uavs) if i != j] for i in range(self.num_uavs)}
        elif self.topology == 'star':
            center = self.num_uavs // 2
            adj = {i: [center] for i in range(self.num_uavs)}
            adj[center] = list(range(self.num_uavs))
            return adj
        return {i: [] for i in range(self.num_uavs)}
    
    def update(self, my_state: np.ndarray, neighbor_states: List[np.ndarray], 
               packet_loss: float = 0.0) -> np.ndarray:
        """
        Update consensus state based on neighbor information.
        
        Args:
            my_state: My current state
            neighbor_states: List of neighbor states
            packet_loss: Probability of communication failure
        
        Returns:
            consensus_state: Agreed state after consensus
        """
        if not neighbor_states:
            return my_state.copy()
        
        if np.random.random() < packet_loss:
            return my_state.copy()
        
        all_states = [my_state] + neighbor_states
        return np.mean(all_states, axis=0)
    
    def get_neighbors(self, uav_id: int) -> List[int]:
        """Get list of neighbor UAV IDs."""
        return self.adjacency.get(uav_id, [])
    
    def compute_consensus_error(self, all_states: List[np.ndarray]) -> float:
        """Compute consensus error (variance)."""
        if not all_states:
            return 0.0
        states_array = np.array(all_states)
        return float(np.var(states_array, axis=0).sum())
    
    def broadcast_state(self, state: np.ndarray, neighbors: List[int], 
                        packet_loss: float = 0.0) -> Dict[int, np.ndarray]:
        """Simulate broadcasting state to neighbors."""
        received = {}
        for neighbor_id in neighbors:
            if np.random.random() >= packet_loss:
                received[neighbor_id] = state.copy()
        return received