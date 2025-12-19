cmake_minimum_required(VERSION 3.8)
project(yolo_realsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OBBDetection.msg"
  "msg/OBBDetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python executables
install(PROGRAMS
  script/yolo_detector.py
  script/pick_controller.py
  script/pick_mouse.py
  script/calibration_script.py
  script/align_and_calibrate.py
  script/visual_servo_calibrate.py
  script/cartesian_calibrate.py
  script/visual_servo_node.py
  script/visual_servo_moveit.py
  script/visual_servo_target.py
  script/visual_servo_velocity.py
  script/visual_servo_ik.py
  script/eye_to_hand_calibration.py
  script/aruco_tf_broadcaster.py
  script/simple_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
