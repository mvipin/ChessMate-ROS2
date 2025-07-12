#!/usr/bin/env python3
"""
ChessMate Project Structure Validation Script

This script validates that all test scripts are in their correct locations
and have the proper paths configured.
"""

import os
import sys

def check_file_exists(filepath, description):
    """Check if a file exists and report status"""
    if os.path.exists(filepath):
        print(f"✅ {description}: {filepath}")
        return True
    else:
        print(f"❌ {description}: {filepath} - NOT FOUND")
        return False

def check_directory_structure():
    """Validate the project directory structure"""
    print("🏗️  Validating ChessMate Project Structure")
    print("=" * 50)
    
    # Get the project root (parent of scripts directory)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    print(f"📁 Project root: {project_root}")
    print(f"📁 Scripts directory: {script_dir}")
    
    # Check main directories
    directories = [
        ("ChessBoard", "ChessBoard controller directory"),
        ("Robot", "Robot controller directory"), 
        ("scripts", "System test scripts directory"),
    ]
    
    all_dirs_exist = True
    for dirname, description in directories:
        dirpath = os.path.join(project_root, dirname)
        if os.path.isdir(dirpath):
            print(f"✅ {description}: {dirname}/")
        else:
            print(f"❌ {description}: {dirname}/ - NOT FOUND")
            all_dirs_exist = False
    
    return all_dirs_exist

def check_test_scripts():
    """Check that all test scripts are in their correct locations"""
    print("\n🧪 Validating Test Scripts")
    print("=" * 30)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Define expected script locations
    scripts = [
        # ChessBoard controller scripts
        (os.path.join(project_root, "ChessBoard", "test_chessboard_controller.py"), 
         "ChessBoard USB test script"),
        (os.path.join(project_root, "ChessBoard", "test_chessboard.sh"), 
         "ChessBoard bash wrapper"),
        
        # Robot controller scripts  
        (os.path.join(project_root, "Robot", "test_robot_controller.py"), 
         "Robot controller test script"),
        (os.path.join(project_root, "Robot", "test_robot.sh"), 
         "Robot bash wrapper"),
        
        # System integration scripts
        (os.path.join(script_dir, "test_complete_game.py"), 
         "Complete game simulation script"),
        (os.path.join(script_dir, "test_chessmate.sh"), 
         "Main test launcher script"),
    ]
    
    all_scripts_exist = True
    for filepath, description in scripts:
        if not check_file_exists(filepath, description):
            all_scripts_exist = False
    
    return all_scripts_exist

def check_controller_code():
    """Check that controller code files exist"""
    print("\n🎮 Validating Controller Code")
    print("=" * 30)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Define expected controller files
    controller_files = [
        # ChessBoard controller
        (os.path.join(project_root, "ChessBoard", "ChessBoard.ino"), 
         "ChessBoard main controller"),
        (os.path.join(project_root, "ChessBoard", "Serial.ino"), 
         "ChessBoard serial communication"),
        (os.path.join(project_root, "ChessBoard", "Utils.h"), 
         "ChessBoard utilities"),
        
        # Robot controller
        (os.path.join(project_root, "Robot", "RobotController.ino"), 
         "Robot main controller"),
        (os.path.join(project_root, "Robot", "RobotUtils.h"), 
         "Robot utilities"),
    ]
    
    all_code_exists = True
    for filepath, description in controller_files:
        if not check_file_exists(filepath, description):
            all_code_exists = False
    
    return all_code_exists

def check_documentation():
    """Check that documentation files exist"""
    print("\n📚 Validating Documentation")
    print("=" * 30)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Define expected documentation files
    docs = [
        (os.path.join(project_root, "ChessBoard", "README.md"), 
         "ChessBoard documentation"),
        (os.path.join(project_root, "Robot", "README.md"), 
         "Robot documentation"),
        (os.path.join(script_dir, "README.md"), 
         "Scripts documentation"),
    ]
    
    all_docs_exist = True
    for filepath, description in docs:
        if not check_file_exists(filepath, description):
            all_docs_exist = False
    
    return all_docs_exist

def main():
    """Main validation function"""
    print("🔍 ChessMate Project Structure Validation")
    print("=" * 50)
    
    # Run all validation checks
    checks = [
        ("Directory Structure", check_directory_structure),
        ("Test Scripts", check_test_scripts),
        ("Controller Code", check_controller_code),
        ("Documentation", check_documentation),
    ]
    
    all_passed = True
    results = []
    
    for check_name, check_func in checks:
        try:
            result = check_func()
            results.append((check_name, result))
            if not result:
                all_passed = False
        except Exception as e:
            print(f"❌ Error during {check_name} check: {e}")
            results.append((check_name, False))
            all_passed = False
    
    # Print summary
    print("\n📊 Validation Summary")
    print("=" * 20)
    
    for check_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {check_name}")
    
    if all_passed:
        print("\n🎉 All validation checks passed!")
        print("✅ Project structure is correctly organized")
        return 0
    else:
        print("\n⚠️  Some validation checks failed")
        print("❌ Please fix the issues above before proceeding")
        return 1

if __name__ == "__main__":
    exit(main())
