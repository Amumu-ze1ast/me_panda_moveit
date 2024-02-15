add_executable(all_codes src/all.cpp)
add_executable(1posegoal src/1posegoal.cpp)
add_executable(2jointspacegoal src/2jointspacegoal.cpp)
add_executable(3pathConstraints src/3pathConstraints.cpp)
add_executable(4cartesianpath src/4cartesianpath.cpp)


target_link_libraries (all_codes ${catkin_LIBRARIES})
target_link_libraries (1posegoal ${catkin_LIBRARIES})
target_link_libraries (2jointspacegoal ${catkin_LIBRARIES})
target_link_libraries (3pathConstraints ${catkin_LIBRARIES})
target_link_libraries (4cartesianpath ${catkin_LIBRARIES})



