cmake_minimum_required(VERSION 3.8)
project(mocha_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClientStats.msg"
  "msg/HeaderPub.msg"
  "msg/SMState.msg"
  "srv/AddUpdateDB.srv"
  "srv/GetDataHeaderDB.srv"
  "srv/SelectDB.srv"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs builtin_interfaces
)

# Install Python package modules as library
install(
  DIRECTORY mocha_core/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install executables as scripts
install(PROGRAMS
  mocha_core/integrate_database.py
  mocha_core/database_server.py
  mocha_core/synchronize_channel.py
  mocha_core/zmq_comm_node.py
  mocha_core/topic_publisher.py
  mocha_core/translator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # Add Python tests
  ament_add_pytest_test(test_database test/test_database.py)
  ament_add_pytest_test(test_database_utils test/test_database_utils.py)
  ament_add_pytest_test(test_zmq_comm_node test/test_zmq_comm_node.py)
  ament_add_pytest_test(test_synchronize_channel test/test_synchronize_channel.py)
  ament_add_pytest_test(test_database_server test/test_database_server.py)
  ament_add_pytest_test(test_delay_2robots test/test_delay_2robots.py)
  ament_add_pytest_test(test_multiple_robots_sync test/test_multiple_robots_sync.py)
  ament_add_pytest_test(test_translator test/test_translator.py)
  ament_add_pytest_test(test_topic_publisher test/test_topic_publisher.py)
endif()

ament_package()
