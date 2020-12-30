add_executable(test_kdtree
    test_kdtree.cpp
    ../include/cluster.hpp)
target_include_directories(test_kdtree
    PRIVATE
        ${PCL_INCLUDE_DIRS})
target_link_libraries(test_kdtree
    PRIVATE
        ${PCL_LIBRARIES})

add_test(NAME test_kdtree COMMAND test_kdtree)
