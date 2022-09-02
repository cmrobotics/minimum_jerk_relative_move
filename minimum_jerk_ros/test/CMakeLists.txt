
set(dependencies
${dependencies}
"cmr_clients_utils"
"cmr_tests_utils"
)

find_package(cmr_clients_utils REQUIRED)
find_package(cmr_tests_utils REQUIRED)


set(TEST_NAME_MOCK collision_checker_mock_test)
ament_add_gtest(${TEST_NAME_MOCK} ${TEST_NAME_MOCK}.cpp)
target_link_libraries(${TEST_NAME_MOCK} ${PROJECT_NAME}_lib)
ament_target_dependencies(${TEST_NAME_MOCK} ${dependencies})
target_include_directories(${TEST_NAME_MOCK} PRIVATE "../include" "include")

set(TEST_NAME_ROT ros_node_test_rot)
ament_add_gtest(${TEST_NAME_ROT} ${TEST_NAME_ROT}.cpp)
target_link_libraries(${TEST_NAME_ROT} ${PROJECT_NAME}_lib)
ament_target_dependencies(${TEST_NAME_ROT} ${dependencies})
target_include_directories(${TEST_NAME_ROT} PRIVATE "../include" "include")

set(TEST_NAME_TRANS ros_node_test_trans)
ament_add_gtest(${TEST_NAME_TRANS} ${TEST_NAME_TRANS}.cpp)
target_link_libraries(${TEST_NAME_TRANS} ${PROJECT_NAME}_lib)
ament_target_dependencies(${TEST_NAME_TRANS} ${dependencies})
target_include_directories(${TEST_NAME_TRANS} PRIVATE "../include" "include")

set(TEST_NAME_COLL ros_node_test_collision)
ament_add_gtest(${TEST_NAME_COLL} ${TEST_NAME_COLL}.cpp)
target_link_libraries(${TEST_NAME_COLL} ${PROJECT_NAME}_lib)
ament_target_dependencies(${TEST_NAME_COLL} ${dependencies})
target_include_directories(${TEST_NAME_COLL} PRIVATE "../include" "include")
