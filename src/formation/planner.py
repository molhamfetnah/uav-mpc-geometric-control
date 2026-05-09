import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class FormationConfig:
    num_uavs: int
    spacing: float = 1.0
    formation_type: str = 'grid'


class FormationPlanner:
    """Plan and manage formation shapes for multi-UAV systems."""
    
    def __init__(self, config: FormationConfig):
        self.config = config
        self.formation_type = config.formation_type
        self.spacing = config.spacing
    
    def compute_offsets(self, center_pos: np.ndarray) -> np.ndarray:
        """Compute target positions for all UAVs relative to center."""
        n = self.config.num_uavs
        
        if self.formation_type == 'grid':
            cols = int(np.ceil(np.sqrt(n)))
            offsets = []
            for i in range(n):
                row = i // cols
                col = i % cols
                offsets.append(np.array([col * self.spacing, row * self.spacing, 0]))
            return np.array(offsets)
        
        elif self.formation_type == 'line':
            start = -((n - 1) * self.spacing) / 2
            return np.array([[start + i * self.spacing, 0, 0] for i in range(n)])
        
        elif self.formation_type == 'circle':
            angles = np.linspace(0, 2*np.pi, n, endpoint=False)
            return np.array([[self.spacing * np.cos(a), self.spacing * np.sin(a), 0] for a in angles])
        
        elif self.formation_type == 'wedge':
            offsets = [np.array([0.0, 0.0, 0.0])]
            for i in range(1, n):
                if i == 1:
                    offsets.append(np.array([self.spacing, 0.0, 0.0]))
                else:
                    angle = (i - 1) / (n - 1) * np.pi / 2
                    offsets.append(np.array([self.spacing * np.cos(angle), self.spacing * np.sin(angle), 0]))
            return np.array(offsets)
        
        return np.zeros((n, 3))
    
    def get_target_position(self, uav_id: int, center_pos: np.ndarray, formation_offsets: np.ndarray) -> np.ndarray:
        """Get target position for specific UAV."""
        if uav_id < len(formation_offsets):
            return center_pos + formation_offsets[uav_id]
        return center_pos.copy()
    
    def transform_formation(self, offsets: np.ndarray, rotation: np.ndarray, 
                           translation: np.ndarray) -> np.ndarray:
        """Apply rotation and translation to formation offsets."""
        rotated = offsets @ rotation.T
        return rotated + translation
    
    def scale_formation(self, offsets: np.ndarray, scale: float) -> np.ndarray:
        """Scale formation by a factor."""
        return offsets * scale
    
    def compute_relative_positions(self, positions: List[np.ndarray]) -> np.ndarray:
        """Compute relative positions between UAVs."""
        n = len(positions)
        rel = []
        for i in range(n):
            for j in range(i+1, n):
                rel.append(positions[j] - positions[i])
        return np.array(rel) if rel else np.zeros((0, 3))