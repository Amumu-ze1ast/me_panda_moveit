add_executable(all_codes src/all.cpp)
add_executable(1posegoal src/1posegoal.cpp)

target_link_libraries (all_codes ${catkin_LIBRARIES})
target_link_libraries (1posegoal ${catkin_LIBRARIES})



