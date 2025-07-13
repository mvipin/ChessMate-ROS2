# ChessMate Description Package

This ROS 2 package contains the robot description files (URDF), 3D models, visualization configurations, and simulation setup for the ChessMate autonomous chess robot system. It provides the complete robot model for visualization, simulation, and motion planning.

## Overview

The package provides:
- **URDF Robot Model**: Complete robot description with joints, links, and sensors
- **3D Meshes**: STL/DAE files for robot visualization and collision detection
- **RViz Configuration**: Pre-configured visualization setups
- **Launch Files**: Robot visualization and simulation launchers
- **Gazebo Integration**: Simulation world and robot spawning

## Package Structure

```
chessmate_description/
├── urdf/
│   ├── chessmate_robot.urdf.xacro    # Main robot description
│   ├── chessmate_arm.urdf.xacro      # Robot arm definition
│   ├── chessmate_base.urdf.xacro     # Base platform definition
│   ├── chessmate_sensors.urdf.xacro  # Sensor definitions
│   └── materials.xacro               # Material definitions
├── meshes/
│   ├── visual/                       # High-quality meshes for visualization
│   │   ├── base_link.stl
│   │   ├── shoulder_link.stl
│   │   ├── upper_arm_link.stl
│   │   ├── forearm_link.stl
│   │   ├── wrist_link.stl
│   │   └── gripper_link.stl
│   └── collision/                    # Simplified meshes for collision detection
│       ├── base_collision.stl
│       ├── arm_collision.stl
│       └── gripper_collision.stl
├── rviz/
│   ├── chessmate_default.rviz        # Default RViz configuration
│   ├── chessmate_planning.rviz       # Motion planning visualization
│   └── chessmate_simulation.rviz     # Simulation visualization
├── launch/
│   ├── display.launch.py             # RViz visualization launcher
│   ├── gazebo.launch.py              # Gazebo simulation launcher
│   └── robot_state_publisher.launch.py  # Robot state publisher
├── config/
│   ├── joint_limits.yaml            # Joint limits configuration
│   └── physical_properties.yaml     # Physical properties
├── worlds/
│   ├── chessmate_world.world         # Gazebo world file
│   └── chess_table.world             # Chess table environment
├── scripts/
│   ├── urdf_validator.py             # URDF validation script
│   └── mesh_converter.py             # Mesh format conversion
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

### ROS 2 Dependencies
- `robot_state_publisher` - Robot state publishing
- `joint_state_publisher` - Joint state management
- `joint_state_publisher_gui` - GUI for joint control
- `rviz2` - 3D visualization
- `gazebo_ros_pkgs` - Gazebo simulation integration
- `xacro` - URDF macro processing

### Optional Dependencies
- `moveit2` - Motion planning integration
- `gazebo_ros2_control` - Hardware interface simulation
- `ros2_control` - Control system integration

## Installation

1. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-robot-state-publisher \
                    ros-humble-joint-state-publisher \
                    ros-humble-joint-state-publisher-gui \
                    ros-humble-rviz2 \
                    ros-humble-gazebo-ros-pkgs \
                    ros-humble-xacro
   ```

2. **Build the package:**
   ```bash
   cd chessmate_dev_ws
   colcon build --packages-select chessmate_description
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### Robot Visualization in RViz

#### Basic Visualization
```bash
# Launch robot model in RViz
ros2 launch chessmate_description display.launch.py

# With joint state publisher GUI for manual control
ros2 launch chessmate_description display.launch.py use_gui:=true

# Load specific RViz configuration
ros2 launch chessmate_description display.launch.py \
    rviz_config:=src/chessmate_description/rviz/chessmate_planning.rviz
```

#### Advanced Visualization
```bash
# With motion planning visualization
ros2 launch chessmate_description display.launch.py \
    config:=planning \
    use_moveit:=true

# With sensor visualization
ros2 launch chessmate_description display.launch.py \
    show_sensors:=true \
    show_camera:=true
```

### Gazebo Simulation

#### Basic Simulation
```bash
# Launch robot in Gazebo
ros2 launch chessmate_description gazebo.launch.py

# With chess table environment
ros2 launch chessmate_description gazebo.launch.py \
    world:=chess_table

# With physics simulation
ros2 launch chessmate_description gazebo.launch.py \
    use_sim_time:=true \
    physics:=ode
```

#### Simulation with Control
```bash
# Launch with ros2_control integration
ros2 launch chessmate_description gazebo.launch.py \
    use_ros2_control:=true \
    controllers:=position_controllers

# With MoveIt integration
ros2 launch chessmate_description gazebo.launch.py \
    use_moveit:=true \
    planning_pipeline:=ompl
```

### Robot State Publisher

```bash
# Standalone robot state publisher
ros2 launch chessmate_description robot_state_publisher.launch.py

# With custom URDF file
ros2 launch chessmate_description robot_state_publisher.launch.py \
    urdf_file:=custom_robot.urdf.xacro

# With specific joint configuration
ros2 launch chessmate_description robot_state_publisher.launch.py \
    joint_config:=home_position
```

## Robot Model Specifications

### Kinematic Chain
The ChessMate robot has a 6-DOF arm with the following joint configuration:

```
Base → Shoulder → Upper Arm → Forearm → Wrist → Gripper
  J1       J2         J3        J4      J5      J6
```

### Joint Specifications

| Joint | Type | Range | Max Velocity | Max Effort |
|-------|------|-------|--------------|------------|
| J1 (Base) | Revolute | ±180° | 90°/s | 10 Nm |
| J2 (Shoulder) | Revolute | -90° to +90° | 90°/s | 15 Nm |
| J3 (Upper Arm) | Revolute | -135° to +135° | 90°/s | 10 Nm |
| J4 (Forearm) | Revolute | ±180° | 120°/s | 5 Nm |
| J5 (Wrist) | Revolute | ±90° | 120°/s | 3 Nm |
| J6 (Gripper) | Prismatic | 0-50mm | 10mm/s | 20 N |

### Physical Properties
- **Total Reach**: 600mm
- **Payload Capacity**: 500g
- **Repeatability**: ±0.5mm
- **Total Weight**: 2.5kg
- **Base Dimensions**: 200mm × 200mm × 100mm

## Configuration Files

### Joint Limits Configuration
Edit `config/joint_limits.yaml`:

```yaml
joint_limits:
  base_joint:
    has_position_limits: true
    min_position: -3.14159
    max_position: 3.14159
    has_velocity_limits: true
    max_velocity: 1.57
    has_acceleration_limits: true
    max_acceleration: 3.14
    
  shoulder_joint:
    has_position_limits: true
    min_position: -1.57
    max_position: 1.57
    has_velocity_limits: true
    max_velocity: 1.57
    
  # ... additional joints
```

### Physical Properties Configuration
Edit `config/physical_properties.yaml`:

```yaml
physical_properties:
  base_link:
    mass: 1.0
    inertia:
      ixx: 0.1
      iyy: 0.1
      izz: 0.1
    material: aluminum
    
  upper_arm_link:
    mass: 0.5
    inertia:
      ixx: 0.05
      iyy: 0.05
      izz: 0.01
    material: carbon_fiber
    
  # ... additional links
```

## URDF Customization

### Adding Custom Sensors
```xml
<!-- Add camera sensor -->
<xacro:include filename="$(find chessmate_description)/urdf/sensors/camera.urdf.xacro"/>

<xacro:camera_sensor 
  parent="wrist_link"
  xyz="0.05 0 0.02"
  rpy="0 0.5 0"/>
```

### Modifying Joint Properties
```xml
<!-- Custom joint definition -->
<joint name="custom_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.01"/>
</joint>
```

### Adding Custom Materials
```xml
<!-- Define custom material -->
<material name="chessmate_blue">
  <color rgba="0.2 0.4 0.8 1.0"/>
</material>

<!-- Apply to link -->
<link name="custom_link">
  <visual>
    <geometry>
      <mesh filename="package://chessmate_description/meshes/visual/custom_part.stl"/>
    </geometry>
    <material name="chessmate_blue"/>
  </visual>
</link>
```

## Visualization Configurations

### RViz Setup
The package includes several pre-configured RViz setups:

1. **Default Configuration** (`chessmate_default.rviz`)
   - Robot model display
   - Joint state visualization
   - TF frame display

2. **Planning Configuration** (`chessmate_planning.rviz`)
   - Motion planning visualization
   - Trajectory display
   - Collision detection

3. **Simulation Configuration** (`chessmate_simulation.rviz`)
   - Gazebo integration
   - Sensor data visualization
   - Real-time joint states

### Custom RViz Configuration
```bash
# Save current RViz configuration
# File → Save Config As → custom_config.rviz

# Load custom configuration
ros2 launch chessmate_description display.launch.py \
    rviz_config:=path/to/custom_config.rviz
```

## Simulation Integration

### Gazebo World Setup
The package includes custom Gazebo worlds:

1. **Chess Table World** - Complete chess playing environment
2. **Testing World** - Simple environment for robot testing
3. **Calibration World** - Precision testing environment

### Physics Configuration
```xml
<!-- Gazebo physics settings -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Testing and Validation

### URDF Validation
```bash
# Validate URDF syntax
ros2 run chessmate_description urdf_validator.py

# Check joint limits
check_urdf src/chessmate_description/urdf/chessmate_robot.urdf.xacro

# Visualize TF tree
ros2 run tf2_tools view_frames.py
```

### Kinematic Testing
```bash
# Test forward kinematics
ros2 run chessmate_description test_kinematics.py --mode forward

# Test inverse kinematics
ros2 run chessmate_description test_kinematics.py --mode inverse

# Workspace analysis
ros2 run chessmate_description analyze_workspace.py
```

### Simulation Testing
```bash
# Test in Gazebo
ros2 launch chessmate_description gazebo.launch.py test_mode:=true

# Run automated tests
python3 src/chessmate_description/test/test_simulation.py
```

## Troubleshooting

### Common Issues

1. **URDF Parse Errors**
   ```bash
   # Check URDF syntax
   check_urdf robot.urdf
   
   # Validate xacro processing
   xacro robot.urdf.xacro > robot.urdf
   ```

2. **Missing Meshes**
   ```bash
   # Check mesh file paths
   find . -name "*.stl" -o -name "*.dae"
   
   # Verify package paths in URDF
   grep -r "package://" urdf/
   ```

3. **RViz Display Issues**
   ```bash
   # Reset RViz configuration
   rm ~/.rviz2/default.rviz
   
   # Check TF frames
   ros2 run tf2_ros tf2_echo base_link end_effector
   ```

4. **Gazebo Simulation Problems**
   ```bash
   # Check Gazebo plugins
   gzserver --verbose
   
   # Verify physics settings
   gz physics -l
   ```

## Contributing

When contributing to the description package:

1. Follow URDF/xacro best practices
2. Maintain consistent naming conventions
3. Include both visual and collision meshes
4. Test changes in both RViz and Gazebo
5. Update documentation for new components
6. Validate URDF syntax before committing

## License

This package is part of the ChessMate project and follows the same licensing terms.
