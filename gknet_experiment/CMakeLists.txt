add_executable(static_grasp src/static_grasp.cpp)
target_link_libraries(static_grasp  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(bin_picking src/bin_picking.cpp)
target_link_libraries(bin_picking  ${catkin_LIBRARIES} ${Boost_LIBRARIES})


