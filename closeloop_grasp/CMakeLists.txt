add_executable(aruco_grasp src/aruco_grasp.cpp)
target_link_libraries(aruco_grasp ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(aruco src/aruco.cpp)
target_link_libraries(aruco ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(real_task src/aruco.cpp)
target_link_libraries(real_task ${catkin_LIBRARIES} ${Boost_LIBRARIES})


