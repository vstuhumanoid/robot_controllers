add_executable(transition_controller
        transition_controller.cpp
        TransitionActionServer/TransitionActionServer.cpp)

target_link_libraries(transition_controller
        ${catkin_LIBRARIES})

add_executable(transition_client transition_client.cpp)

target_link_libraries(transition_client
        ${catkin_LIBRARIES})