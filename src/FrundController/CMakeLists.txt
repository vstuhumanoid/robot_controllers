find_package(Qt5Core REQUIRED)

add_executable(frund_controller
        frund_controller.cpp
        FrundModelActionServer/FrundModelActionServer.cpp
        FrundPacketConverter/FrundPacketConverter.cpp
        FrundGateway/FrundGateway.cpp)

add_dependencies(frund_controller robot_controllers_generate_messages_cpp)

target_link_libraries(frund_controller
        ${catkin_LIBRARIES}
        joint_name_converter)

qt5_use_modules(frund_controller Core Network)



add_executable(frund_client frund_client.cpp)

add_dependencies(frund_client robot_controllers_generate_messages_cpp)

target_link_libraries(frund_client ${catkin_LIBRARIES})