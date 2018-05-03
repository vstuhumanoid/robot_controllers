add_executable(frund_controller
        frund_controller.cpp
        FrundModelActionServer/FrundModelActionServer.cpp
        FrundPacketConverter/FrundPacketConverter.cpp)

add_dependencies(frund_controller robot_controllers_generate_messages_cpp)

target_link_libraries(frund_controller
        ${catkin_LIBRARIES}
        joint_name_converter)


#add_executable(transition_client transition_client.cpp)

#target_link_libraries(transition_client
#        ${catkin_LIBRARIES})