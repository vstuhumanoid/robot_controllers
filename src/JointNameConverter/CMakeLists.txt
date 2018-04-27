project(joint_name_converter)

add_library(${PROJECT_NAME} STATIC
        JointNameConverter.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})