project(TEST_API)

add_executable(test_api_node
    test_api_node.cpp
    test_api.cpp
    test_api.h
)

target_link_libraries(test_api_node
    ORB_SLAM3_API
    ${EIGEN3_LIBS}
    ORB_SLAM3
)
