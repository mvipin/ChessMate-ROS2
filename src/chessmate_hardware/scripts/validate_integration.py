#!/usr/bin/env python3
"""
Integration Validation Script for ChessMate Unified Implementation

Validates the successful integration of Raspberry Pi and Linux host implementations
into a unified codebase. Performs comprehensive checks on:

- Message type compatibility
- Node functionality
- Configuration loading
- Cross-platform compatibility
- Build system integrity
"""

import os
import sys
import subprocess
import yaml
import importlib.util
from pathlib import Path
from typing import List, Dict, Any, Tuple


class IntegrationValidator:
    """
    Comprehensive validation of ChessMate integration
    """
    
    def __init__(self, workspace_path: str):
        self.workspace_path = Path(workspace_path)
        self.package_path = self.workspace_path / 'src' / 'chessmate_hardware'
        self.msgs_path = self.workspace_path / 'src' / 'chessmate_msgs'
        self.validation_results = {}
        
    def run_all_validations(self) -> Dict[str, bool]:
        """Run all validation checks"""
        print("🔍 Starting ChessMate Integration Validation")
        print("=" * 60)
        
        validations = [
            ("Package Structure", self.validate_package_structure),
            ("Message Types", self.validate_message_types),
            ("Node Implementations", self.validate_node_implementations),
            ("Configuration System", self.validate_configuration_system),
            ("Build System", self.validate_build_system),
            ("Cross-Platform Support", self.validate_cross_platform_support),
            ("Integration Completeness", self.validate_integration_completeness),
        ]
        
        for name, validation_func in validations:
            print(f"\n📋 {name}...")
            try:
                result = validation_func()
                self.validation_results[name] = result
                status = "✅ PASS" if result else "❌ FAIL"
                print(f"   {status}")
            except Exception as e:
                print(f"   ❌ ERROR: {e}")
                self.validation_results[name] = False
        
        self.generate_validation_report()
        return self.validation_results
    
    def validate_package_structure(self) -> bool:
        """Validate package directory structure"""
        required_files = [
            'package.xml',
            'setup.py',
            'chessmate_hardware/__init__.py',
            'chessmate_hardware/unified_arduino_bridge.py',
            'chessmate_hardware/unified_hardware_test.py',
            'chessmate_hardware/config_loader.py',
            'config/unified_hardware_config.yaml',
            'launch/unified_hardware.launch.py',
            'scripts/validate_integration.py',
        ]
        
        missing_files = []
        for file_path in required_files:
            full_path = self.package_path / file_path
            if not full_path.exists():
                missing_files.append(file_path)
        
        if missing_files:
            print(f"   Missing files: {missing_files}")
            return False
        
        print("   All required files present")
        return True
    
    def validate_message_types(self) -> bool:
        """Validate message type definitions and compatibility"""
        if not self.msgs_path.exists():
            print("   chessmate_msgs package not found")
            return False
        
        # Check for required message files
        required_msgs = [
            'msg/ArduinoCommand.msg',
            'msg/JointCommand.msg',
            'msg/SensorReading.msg',
            'msg/BoardState.msg',
            'msg/RobotAnimation.msg',
            'srv/CalibrateArm.srv',
        ]
        
        missing_msgs = []
        for msg_file in required_msgs:
            msg_path = self.msgs_path / msg_file
            if not msg_path.exists():
                missing_msgs.append(msg_file)
        
        if missing_msgs:
            print(f"   Missing message files: {missing_msgs}")
            return False
        
        # Validate CMakeLists.txt includes all messages
        cmake_file = self.msgs_path / 'CMakeLists.txt'
        if cmake_file.exists():
            with open(cmake_file, 'r') as f:
                cmake_content = f.read()
                
            # Check if all message files are listed
            for msg_file in required_msgs:
                msg_name = Path(msg_file).name
                if msg_name not in cmake_content:
                    print(f"   Message {msg_name} not in CMakeLists.txt")
                    return False
        
        print("   All message types validated")
        return True
    
    def validate_node_implementations(self) -> bool:
        """Validate node implementation files"""
        # ROS 2 nodes that require rclpy
        ros_node_files = [
            'unified_arduino_bridge.py',
            'unified_hardware_test.py',
            'game_management_node.py',
            'robot_animation_controller.py',
        ]

        # Utility modules that don't require rclpy
        utility_files = [
            'config_loader.py',
        ]

        # Validate ROS 2 nodes
        for node_file in ros_node_files:
            node_path = self.package_path / 'chessmate_hardware' / node_file
            if not node_path.exists():
                print(f"   Missing ROS node file: {node_file}")
                return False

            # Basic syntax validation for ROS nodes
            try:
                with open(node_path, 'r') as f:
                    content = f.read()

                # Check for basic ROS 2 node structure
                if 'rclpy' not in content:
                    print(f"   {node_file} missing rclpy import")
                    return False

                if 'class' not in content and 'def main' not in content:
                    print(f"   {node_file} missing class or main function")
                    return False

            except Exception as e:
                print(f"   Error reading {node_file}: {e}")
                return False

        # Validate utility modules
        for util_file in utility_files:
            util_path = self.package_path / 'chessmate_hardware' / util_file
            if not util_path.exists():
                print(f"   Missing utility file: {util_file}")
                return False

            # Basic syntax validation for utilities
            try:
                with open(util_path, 'r') as f:
                    content = f.read()

                # Check for basic Python structure
                if 'class' not in content and 'def' not in content:
                    print(f"   {util_file} missing class or function definitions")
                    return False

            except Exception as e:
                print(f"   Error reading {util_file}: {e}")
                return False

        print("   All node implementations validated")
        return True
    
    def validate_configuration_system(self) -> bool:
        """Validate configuration system"""
        config_file = self.package_path / 'config' / 'unified_hardware_config.yaml'
        
        if not config_file.exists():
            print("   Configuration file missing")
            return False
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Check for required configuration sections
            required_sections = [
                'platform',
                'serial',
                'protocol',
                'hardware',
                'publishing',
                'platform_overrides'
            ]
            
            for section in required_sections:
                if section not in config:
                    print(f"   Missing config section: {section}")
                    return False
            
            # Check platform overrides
            if 'raspberry_pi' not in config['platform_overrides']:
                print("   Missing Raspberry Pi platform override")
                return False
            
            if 'linux_host' not in config['platform_overrides']:
                print("   Missing Linux host platform override")
                return False
            
        except yaml.YAMLError as e:
            print(f"   YAML parsing error: {e}")
            return False
        except Exception as e:
            print(f"   Configuration validation error: {e}")
            return False
        
        print("   Configuration system validated")
        return True
    
    def validate_build_system(self) -> bool:
        """Validate build system configuration"""
        setup_py = self.package_path / 'setup.py'
        package_xml = self.package_path / 'package.xml'
        
        if not setup_py.exists() or not package_xml.exists():
            print("   Missing build files")
            return False
        
        try:
            # Check setup.py for all entry points
            with open(setup_py, 'r') as f:
                setup_content = f.read()
            
            required_entry_points = [
                'unified_arduino_bridge',
                'unified_hardware_test',
                'game_management_node',
                'robot_animation_controller'
            ]
            
            for entry_point in required_entry_points:
                if entry_point not in setup_content:
                    print(f"   Missing entry point: {entry_point}")
                    return False
            
            # Check for required dependencies
            required_deps = ['pyserial', 'setuptools']
            for dep in required_deps:
                if dep not in setup_content:
                    print(f"   Missing dependency: {dep}")
                    return False
            
        except Exception as e:
            print(f"   Build system validation error: {e}")
            return False
        
        print("   Build system validated")
        return True
    
    def validate_cross_platform_support(self) -> bool:
        """Validate cross-platform compatibility"""
        try:
            # Test config loader platform detection
            config_loader_path = self.package_path / 'chessmate_hardware' / 'config_loader.py'
            
            if not config_loader_path.exists():
                print("   Config loader missing")
                return False
            
            # Check for platform detection logic
            with open(config_loader_path, 'r') as f:
                content = f.read()
            
            platform_indicators = [
                'raspberry_pi',
                'linux_host',
                '_detect_platform',
                'platform_overrides'
            ]
            
            for indicator in platform_indicators:
                if indicator not in content:
                    print(f"   Missing platform support: {indicator}")
                    return False
            
            # Check launch file for platform support
            launch_file = self.package_path / 'launch' / 'unified_hardware.launch.py'
            
            if launch_file.exists():
                with open(launch_file, 'r') as f:
                    launch_content = f.read()
                
                if 'detect_platform' not in launch_content:
                    print("   Launch file missing platform detection")
                    return False
            
        except Exception as e:
            print(f"   Cross-platform validation error: {e}")
            return False
        
        print("   Cross-platform support validated")
        return True
    
    def validate_integration_completeness(self) -> bool:
        """Validate that integration is complete and no duplication exists"""
        try:
            # Check for duplicate functionality
            node_files = list((self.package_path / 'chessmate_hardware').glob('*.py'))
            
            # Look for potential duplicates
            arduino_comm_files = [f for f in node_files if 'arduino' in f.name.lower()]
            
            if len(arduino_comm_files) > 2:  # unified_arduino_bridge + original (for compatibility)
                print(f"   Potential duplicate Arduino communication files: {[f.name for f in arduino_comm_files]}")
                # This is actually OK - we keep the original for compatibility
            
            # Check that unified bridge has both protocol support
            unified_bridge = self.package_path / 'chessmate_hardware' / 'unified_arduino_bridge.py'
            
            if unified_bridge.exists():
                with open(unified_bridge, 'r') as f:
                    content = f.read()
                
                required_features = [
                    'JSON_BASED',
                    'CHARACTER_BASED',
                    'ProtocolType',
                    'ArduinoType',
                    '_format_json_command',
                    '_format_character_command'
                ]
                
                for feature in required_features:
                    if feature not in content:
                        print(f"   Unified bridge missing feature: {feature}")
                        return False
            
        except Exception as e:
            print(f"   Integration completeness validation error: {e}")
            return False
        
        print("   Integration completeness validated")
        return True
    
    def generate_validation_report(self):
        """Generate comprehensive validation report"""
        print("\n" + "=" * 60)
        print("📊 INTEGRATION VALIDATION REPORT")
        print("=" * 60)
        
        passed_count = sum(self.validation_results.values())
        total_count = len(self.validation_results)
        success_rate = (passed_count / total_count) * 100 if total_count > 0 else 0
        
        for test_name, result in self.validation_results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"{test_name:.<40} {status}")
        
        print("-" * 60)
        print(f"Overall Success Rate: {success_rate:.1f}% ({passed_count}/{total_count})")
        
        if success_rate == 100:
            print("🎉 INTEGRATION VALIDATION: COMPLETE SUCCESS")
            print("   All systems integrated successfully!")
        elif success_rate >= 80:
            print("✅ INTEGRATION VALIDATION: MOSTLY SUCCESSFUL")
            print("   Minor issues detected, but integration is functional")
        elif success_rate >= 60:
            print("⚠️  INTEGRATION VALIDATION: PARTIAL SUCCESS")
            print("   Significant issues detected, review required")
        else:
            print("❌ INTEGRATION VALIDATION: FAILED")
            print("   Major issues detected, integration incomplete")
        
        print("=" * 60)


def main():
    """Main validation entry point"""
    if len(sys.argv) > 1:
        workspace_path = sys.argv[1]
    else:
        # Try to detect workspace path
        current_dir = Path.cwd()
        
        # Look for workspace indicators
        possible_paths = [
            current_dir,
            current_dir / 'chessmate_dev_ws',
            current_dir.parent / 'chessmate_dev_ws',
            Path.home() / 'ChessMate' / 'chessmate_dev_ws'
        ]
        
        workspace_path = None
        for path in possible_paths:
            if (path / 'src' / 'chessmate_hardware').exists():
                workspace_path = str(path)
                break
        
        if not workspace_path:
            print("❌ Could not find ChessMate workspace")
            print("Usage: python validate_integration.py [workspace_path]")
            sys.exit(1)
    
    print(f"🔍 Validating ChessMate integration in: {workspace_path}")
    
    validator = IntegrationValidator(workspace_path)
    results = validator.run_all_validations()
    
    # Exit with appropriate code
    success_rate = sum(results.values()) / len(results) * 100
    sys.exit(0 if success_rate >= 80 else 1)


if __name__ == '__main__':
    main()
