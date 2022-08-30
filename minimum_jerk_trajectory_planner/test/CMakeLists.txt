
set(dependencies
${dependencies}
"cmr_clients_utils"
"cmr_tests_utils"
)

find_package(cmr_clients_utils REQUIRED)
find_package(cmr_tests_utils REQUIRED)

set(TEST_NAME_ROT library_test_rot)
ament_add_gtest(${TEST_NAME_ROT} ${TEST_NAME_ROT}.cpp)
target_link_libraries(${TEST_NAME_ROT} ${PROJECT_NAME})
ament_target_dependencies(${TEST_NAME_ROT} ${dependencies})
target_include_directories(${TEST_NAME_ROT} PRIVATE "../include" "include")

set(TEST_NAME_TRANS library_test_trans)
ament_add_gtest(${TEST_NAME_TRANS} ${TEST_NAME_TRANS}.cpp)
target_link_libraries(${TEST_NAME_TRANS} ${PROJECT_NAME})
ament_target_dependencies(${TEST_NAME_TRANS} ${dependencies})
target_include_directories(${TEST_NAME_TRANS} PRIVATE "../include" "include")
