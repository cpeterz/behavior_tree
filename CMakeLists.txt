cmake_minimum_required(VERSION 3.5)
project(behavior_tree)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp)
find_package(base_interfaces)
find_package(OpenCV)
find_package(libbase REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(behaviortree src/BehaviorTree.cpp)

add_definitions(-DBT_XML="${CMAKE_CURRENT_LIST_DIR}/config/BehaviorTree2.xml")
add_definitions(-DBT_YAML="${CMAKE_CURRENT_LIST_DIR}/config/BehaviorTree.yaml")

target_link_libraries(behaviortree BT::behaviortree_cpp ${OpenCV_LIBS})

ament_target_dependencies(behaviortree 
  rclcpp
  base_interfaces
  libbase
)

install(TARGETS
  behaviortree
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
