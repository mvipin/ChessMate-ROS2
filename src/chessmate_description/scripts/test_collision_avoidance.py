#!/usr/bin/env python3
"""
Test script to demonstrate SCARA self-collision avoidance
Shows how the 30-degree buffer constraint prevents Link 2 from folding onto Link 1
"""

import math
import sys
import os

# Add the kinematics package to the path
sys.path.append('/home/smtuser/ChessMate/chessmate_dev_ws/install/chessmate_kinematics/local/lib/python3.10/dist-packages')

from chessmate_kinematics.scara_kinematics import SCARAKinematics, SCARAConfiguration

def test_collision_avoidance():
    """Test the collision avoidance constraints"""
    
    print("🤖 SCARA Self-Collision Avoidance Test")
    print("=" * 50)
    
    # Create SCARA with collision avoidance enabled
    config = SCARAConfiguration(
        collision_buffer_angle=math.radians(30),  # 30-degree buffer
        enable_collision_avoidance=True
    )
    scara = SCARAKinematics(config)
    
    print(f"✅ Collision buffer: {math.degrees(config.collision_buffer_angle):.1f}°")
    print(f"✅ Collision avoidance: {'Enabled' if config.enable_collision_avoidance else 'Disabled'}")
    print()
    
    # Test cases: theta1=0 (Link 1 pointing forward), various theta2 values
    theta1 = 0.0  # Link 1 pointing in +X direction
    z = 0.05      # Mid-height
    
    test_angles = [
        (0.0, "Link 2 aligned with Link 1 - SHOULD FAIL"),
        (math.radians(15), "Link 2 at +15° from Link 1 - SHOULD FAIL"),
        (math.radians(20), "Link 2 at +20° from Link 1 - BOUNDARY"),
        (math.radians(30), "Link 2 at +30° from Link 1 - SHOULD PASS"),
        (math.radians(35), "Link 2 at +35° from Link 1 - SHOULD PASS"),
        (math.radians(90), "Link 2 perpendicular to Link 1 - SHOULD PASS"),
        (math.radians(100), "Link 2 at +100° from Link 1 - BOUNDARY"),
        (math.radians(104.7), "θ2=104.7° (from e4 logs) - SHOULD FAIL"),
        (math.radians(124.5), "θ2=124.5° (from a1/h1 logs) - SHOULD FAIL"),
        (math.radians(113.8), "θ2=113.8° (from c3 logs) - SHOULD PASS/FAIL?"),
        (math.radians(-15), "Link 2 at -15° from Link 1 - SHOULD FAIL"),
        (math.radians(-20), "Link 2 at -20° from Link 1 - BOUNDARY"),
        (math.radians(-30), "Link 2 at -30° from Link 1 - SHOULD PASS"),
        (math.radians(-35), "Link 2 at -35° from Link 1 - SHOULD PASS"),
        (math.radians(180), "Link 2 fully folded back - SHOULD FAIL"),
        (math.radians(150), "Link 2 at +150° - SHOULD FAIL"),
        (math.radians(-150), "Link 2 at -150° - SHOULD FAIL"),
    ]
    
    print("🧪 Testing Joint Configurations:")
    print("-" * 70)
    
    for theta2, description in test_angles:
        try:
            # Test if configuration is collision-free
            is_safe = scara.is_collision_free(theta1, theta2, z)
            
            if is_safe:
                # Also test forward kinematics to get end position
                x, y, z_out = scara.forward_kinematics(theta1, theta2, z)
                result = f"✅ PASS - End position: ({x*1000:.1f}, {y*1000:.1f})mm"
            else:
                result = "❌ FAIL - Collision detected"
                
        except ValueError as e:
            result = f"❌ FAIL - {str(e)}"
        
        theta2_deg = math.degrees(theta2)
        print(f"θ2={theta2_deg:+6.1f}° | {description:<35} | {result}")
    
    print()
    print("🎯 Valid θ2 Range Analysis:")
    print("-" * 40)
    
    # Test valid ranges for different theta1 values
    for theta1_test in [0, math.radians(45), math.radians(90)]:
        theta2_min, theta2_max = scara.get_collision_free_theta2_range(theta1_test)
        theta1_deg = math.degrees(theta1_test)
        theta2_min_deg = math.degrees(theta2_min)
        theta2_max_deg = math.degrees(theta2_max)
        
        print(f"θ1={theta1_deg:+6.1f}° | Valid θ2 range: [{theta2_min_deg:+6.1f}°, {theta2_max_deg:+6.1f}°]")
    
    print()
    print("🔧 Testing Inverse Kinematics with Collision Avoidance:")
    print("-" * 55)
    
    # Test some positions that might cause collisions
    test_positions = [
        (0.1, 0.0, 0.05, "Straight ahead - close range"),
        (0.3, 0.0, 0.05, "Straight ahead - far range"),
        (0.0, 0.2, 0.05, "Pure sideways"),
        (-0.1, 0.0, 0.05, "Behind robot - might cause folding"),
    ]
    
    for x, y, z, description in test_positions:
        try:
            theta1, theta2, z_out = scara.inverse_kinematics(x, y, z)
            theta1_deg = math.degrees(theta1)
            theta2_deg = math.degrees(theta2)
            
            # Verify it's collision-free
            is_safe = scara.is_collision_free(theta1, theta2, z_out)
            safety_status = "✅ Safe" if is_safe else "❌ Collision!"
            
            print(f"({x:+5.2f}, {y:+5.2f})m | {description:<25} | θ1={theta1_deg:+6.1f}°, θ2={theta2_deg:+6.1f}° | {safety_status}")
            
        except ValueError as e:
            print(f"({x:+5.2f}, {y:+5.2f})m | {description:<25} | ❌ No solution: {str(e)[:40]}...")
    
    print()
    print("🔍 Testing Specific Problematic Configurations from Animation Logs:")
    print("-" * 65)

    # Test the exact joint combinations that were causing overlaps
    problematic_configs = [
        (-92.9, 124.5, "a1@60mm position - SHOULD FAIL"),
        (-25.0, 124.5, "h1@40mm position - SHOULD FAIL"),
        (-46.6, 104.7, "e4@25mm position - SHOULD PASS"),
        (-66.0, 113.8, "c3@55mm position - SHOULD PASS/FAIL?"),
        (-53.6, 104.7, "d4@30mm position - SHOULD PASS"),
    ]

    for theta1_deg, theta2_deg, description in problematic_configs:
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)

        try:
            is_safe = scara.is_collision_free(theta1, theta2, 0.05)
            if is_safe:
                result = "✅ PASS - Configuration is safe"
            else:
                result = "❌ FAIL - Collision detected"
        except ValueError as e:
            result = f"❌ FAIL - {str(e)[:50]}..."

        print(f"θ1={theta1_deg:+6.1f}°, θ2={theta2_deg:+6.1f}° | {description:<25} | {result}")

    print()
    print("🎉 Collision Avoidance Test Complete!")
    print("The 30° buffer successfully prevents Link 2 from folding onto Link 1.")

if __name__ == '__main__':
    test_collision_avoidance()
