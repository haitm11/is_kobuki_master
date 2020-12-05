##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################


# dang ky file thuc thi la "kobuki_controller.cpp" thi chi khi sua file "kobuki_controller.cpp"
# catkin_make moi chay
add_executable(is_kobuki_node controller/kobuki_controller.cpp main.cpp)

add_dependencies(is_kobuki_node kobuki_msgs_gencpp is_kobuki)

target_link_libraries(is_kobuki_node ${catkin_LIBRARIES})

install(TARGETS is_kobuki_node is_kobuki_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


