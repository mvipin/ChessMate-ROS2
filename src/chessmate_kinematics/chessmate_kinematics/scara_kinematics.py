#!/usr/bin/env python3
"""
SCARA Kinematics Implementation for ChessMate
Provides forward and inverse kinematics for 3-DOF SCARA arm configuration
"""

import math
import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass


@dataclass
class SCARAConfiguration:
    """SCARA arm configuration parameters"""
    l1: float = 0.202  # Link 1 length (meters)
    l2: float = 0.190  # Link 2 length (meters)
    z_min: float = 0.0  # Minimum Z height (meters)
    z_max: float = 0.100  # Maximum Z height (meters)
    
    # Joint limits (radians)
    theta1_min: float = -math.pi
    theta1_max: float = math.pi
    theta2_min: float = -math.pi
    theta2_max: float = math.pi


class SCARAKinematics:
    """
    SCARA (Selective Compliance Assembly Robot Arm) Kinematics Solver
    
    Coordinate System:
    - X: Forward direction from robot base
    - Y: Left direction from robot base  
    - Z: Upward direction (vertical axis)
    - Joint 1 (theta1): Base rotation
    - Joint 2 (theta2): Elbow rotation
    - Joint 3 (z): Vertical translation
    """
    
    def __init__(self, config: Optional[SCARAConfiguration] = None):
        """Initialize SCARA kinematics with configuration parameters"""
        self.config = config or SCARAConfiguration()
        
        # Workspace limits
        self.workspace_radius_max = self.config.l1 + self.config.l2
        self.workspace_radius_min = abs(self.config.l1 - self.config.l2)
        
    def forward_kinematics(self, theta1: float, theta2: float, z: float) -> Tuple[float, float, float]:
        """
        Convert joint angles to Cartesian coordinates
        
        Args:
            theta1: Base joint angle (radians)
            theta2: Elbow joint angle (radians)  
            z: Vertical position (meters)
            
        Returns:
            Tuple of (x, y, z) coordinates in meters
            
        Raises:
            ValueError: If joint limits are exceeded
        """
        # Validate joint limits
        self._validate_joint_limits(theta1, theta2, z)
        
        # Forward kinematics equations
        x = self.config.l1 * math.cos(theta1) + self.config.l2 * math.cos(theta1 + theta2)
        y = self.config.l1 * math.sin(theta1) + self.config.l2 * math.sin(theta1 + theta2)
        
        return (x, y, z)
    
    def inverse_kinematics(self, x: float, y: float, z: float, 
                          elbow_up: bool = True) -> Tuple[float, float, float]:
        """
        Convert Cartesian coordinates to joint angles
        
        Args:
            x: X coordinate (meters)
            y: Y coordinate (meters)
            z: Z coordinate (meters)
            elbow_up: True for elbow-up configuration, False for elbow-down
            
        Returns:
            Tuple of (theta1, theta2, z) joint values
            
        Raises:
            ValueError: If target is outside workspace or invalid
        """
        # Validate Z limits
        if not (self.config.z_min <= z <= self.config.z_max):
            raise ValueError(f"Z coordinate {z:.3f} outside limits [{self.config.z_min:.3f}, {self.config.z_max:.3f}]")
        
        # Calculate distance from origin to target
        r = math.sqrt(x*x + y*y)
        
        # Check workspace limits
        if r > self.workspace_radius_max:
            raise ValueError(f"Target distance {r:.3f}m exceeds maximum reach {self.workspace_radius_max:.3f}m")
        if r < self.workspace_radius_min:
            raise ValueError(f"Target distance {r:.3f}m below minimum reach {self.workspace_radius_min:.3f}m")
        
        # Handle singularity at origin
        if r < 1e-6:
            return (0.0, 0.0, z)
        
        # Solve for theta2 using law of cosines
        cos_theta2 = (r*r - self.config.l1*self.config.l1 - self.config.l2*self.config.l2) / (2*self.config.l1*self.config.l2)
        
        # Check if solution exists
        if abs(cos_theta2) > 1.0:
            raise ValueError(f"No IK solution exists for target ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Calculate theta2 (elbow angle)
        sin_theta2 = math.sqrt(1 - cos_theta2*cos_theta2)
        if not elbow_up:
            sin_theta2 = -sin_theta2
        theta2 = math.atan2(sin_theta2, cos_theta2)
        
        # Calculate theta1 (base angle)
        alpha = math.atan2(y, x)
        beta = math.atan2(self.config.l2 * sin_theta2, self.config.l1 + self.config.l2 * cos_theta2)
        theta1 = alpha - beta
        
        # Normalize angles to [-pi, pi]
        theta1 = self._normalize_angle(theta1)
        theta2 = self._normalize_angle(theta2)
        
        return (theta1, theta2, z)
    
    def get_workspace_boundary(self, num_points: int = 100) -> List[Tuple[float, float]]:
        """
        Generate points on the workspace boundary for visualization
        
        Args:
            num_points: Number of boundary points to generate
            
        Returns:
            List of (x, y) coordinates on workspace boundary
        """
        boundary_points = []
        
        # Outer boundary (maximum reach)
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = self.workspace_radius_max * math.cos(angle)
            y = self.workspace_radius_max * math.sin(angle)
            boundary_points.append((x, y))
        
        # Inner boundary (minimum reach) - only if significant
        if self.workspace_radius_min > 0.01:
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                x = self.workspace_radius_min * math.cos(angle)
                y = self.workspace_radius_min * math.sin(angle)
                boundary_points.append((x, y))
        
        return boundary_points
    
    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """
        Check if a Cartesian position is reachable
        
        Args:
            x, y, z: Target coordinates
            
        Returns:
            True if position is reachable, False otherwise
        """
        try:
            self.inverse_kinematics(x, y, z)
            return True
        except ValueError:
            return False
    
    def _validate_joint_limits(self, theta1: float, theta2: float, z: float):
        """Validate joint angles against limits"""
        if not (self.config.theta1_min <= theta1 <= self.config.theta1_max):
            raise ValueError(f"theta1 {theta1:.3f} outside limits [{self.config.theta1_min:.3f}, {self.config.theta1_max:.3f}]")
        if not (self.config.theta2_min <= theta2 <= self.config.theta2_max):
            raise ValueError(f"theta2 {theta2:.3f} outside limits [{self.config.theta2_min:.3f}, {self.config.theta2_max:.3f}]")
        if not (self.config.z_min <= z <= self.config.z_max):
            raise ValueError(f"z {z:.3f} outside limits [{self.config.z_min:.3f}, {self.config.z_max:.3f}]")
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
