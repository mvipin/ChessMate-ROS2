#!/usr/bin/env python3

import sys
import os
import math

# Add the kinematics package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src/chessmate_kinematics'))

from chessmate_kinematics.scara_kinematics import SCARAKinematics, SCARAConfiguration

def test_direct_collision():
    print("🔍 Direct Collision Detection Test")
    print("=" * 50)
    
    # Create SCARA with collision avoidance enabled
    config = SCARAConfiguration(
        collision_buffer_angle=math.radians(30),
        enable_collision_avoidance=True
    )
    scara = SCARAKinematics(config)
    
    print(f"✅ Collision avoidance enabled: {config.enable_collision_avoidance}")
    print(f"✅ Buffer angle: {math.degrees(config.collision_buffer_angle):.1f}°")
    print()
    
    # Test the relaxed boundaries
    test_cases = [
        (0.0, 19.0, "Should FAIL - below 20° limit"),
        (0.0, 20.0, "Should PASS - at 20° limit"),
        (0.0, 21.0, "Should PASS - above 20° limit"),
        (0.0, 99.0, "Should PASS - below 100° limit"),
        (0.0, 100.0, "Should PASS - at 100° limit"),
        (0.0, 101.0, "Should FAIL - above 100° limit"),
        (0.0, 104.7, "Should FAIL - from e4 logs (above 100°)"),
        (0.0, 124.5, "Should FAIL - from a1/h1 logs (above 100°)"),
    ]
    
    for theta1_deg, theta2_deg, description in test_cases:
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)
        
        print(f"Testing θ1={theta1_deg:.1f}°, θ2={theta2_deg:.1f}° - {description}")
        
        try:
            # Test collision detection directly
            scara._validate_collision_avoidance(theta1, theta2)
            print(f"  ✅ PASS - No collision detected")
        except ValueError as e:
            print(f"  ❌ FAIL - {e}")
        
        try:
            # Test is_collision_free method
            is_safe = scara.is_collision_free(theta1, theta2, 0.05)
            print(f"  is_collision_free(): {is_safe}")
        except Exception as e:
            print(f"  is_collision_free() error: {e}")
        
        print()

if __name__ == "__main__":
    test_direct_collision()
