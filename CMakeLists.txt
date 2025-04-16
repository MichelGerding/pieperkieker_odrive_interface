cmake_minimum_required(VERSION 3.8)
project(odrive_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/OdriveCommand.msg"
        "msg/OdriveFeedback.msg"
        DEPENDENCIES std_msgs
)

add_executable(odrive_serial_node src/odrive_serial_node.cpp)
target_include_directories(odrive_serial_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${LIBSERIALPORT_INCLUDE_DIRS}
)

target_link_libraries(odrive_serial_node
        ${LIBSERIALPORT_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(odrive_serial_node
        ${cpp_typesupport_target})


ament_target_dependencies(odrive_serial_node
        rclcpp
        std_msgs
)

# Install the executable
install(TARGETS
        odrive_serial_node
        DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, if any
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        OPTIONAL
)

# Install include directory, if any
install(DIRECTORY
        include/
        DESTINATION include/${PROJECT_NAME}
        OPTIONAL
)

ament_package()