add_executable(play_file_controller
        play_file_controller.cpp
        PlayFileActionServer/PlayFileActionServer.cpp)

add_dependencies(play_file_controller robot_controllers_generate_messages_cpp)

target_link_libraries(play_file_controller
        ${catkin_LIBRARIES}
        joint_name_converter)

add_executable(play_file_client play_file_client.cpp)

add_dependencies(play_file_client robot_controllers_generate_messages_cpp)

target_link_libraries(play_file_client
        ${catkin_LIBRARIES})