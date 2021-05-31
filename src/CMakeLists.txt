ADD_EXECUTABLE (simviz_test simviz_test.cpp ${CS225A_COMMON_SOURCE})
ADD_LIBRARY (volleybot_controller VolleybotController_test.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_test controller_test.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
#TARGET_LINK_LIBRARIES (controller_test ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_test ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_test volleybot_controller ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
