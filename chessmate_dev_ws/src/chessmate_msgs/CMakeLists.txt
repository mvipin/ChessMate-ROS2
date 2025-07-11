cmake_minimum_required(VERSION 3.8)
project(chessmate_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChessMove.msg"
  "msg/ChessPiece.msg"
  "msg/BoardState.msg"
  "msg/GripperCommand.msg"
  "msg/JointCommand.msg"
  "msg/RobotStatus.msg"
  "msg/RotaryEncoderEvent.msg"
  "msg/LCDCommand.msg"
  "msg/ArduinoCommand.msg"
  "msg/RobotAnimation.msg"
  "msg/SensorReading.msg"
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
  "srv/ChessSquareKinematics.srv"
  "srv/GetBestMove.srv"
  "srv/EvaluatePosition.srv"
  "srv/ValidateMove.srv"
  "srv/UpdateGameState.srv"
  "srv/CalibrateArm.srv"
  "action/ExecuteChessMove.action"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs action_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
