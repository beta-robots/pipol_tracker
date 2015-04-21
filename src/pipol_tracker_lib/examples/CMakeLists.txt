# create an example application
# ADD_EXECUTABLE(pipol_tracker_test pipol_tracker_test.cpp)
# ADD_EXECUTABLE(appearance_test appearance_test.cpp)
# ADD_EXECUTABLE(appearance_metrics_test appearance_metrics_test.cpp)
# link necessary libraries
# TARGET_LINK_LIBRARIES(pipol_tracker_test pipol_tracker)
# TARGET_LINK_LIBRARIES(appearance_test pipol_tracker)
# TARGET_LINK_LIBRARIES(appearance_metrics_test people_tracking_rai)

#test matrix
ADD_EXECUTABLE(test_matrix test_matrix.cpp)

#test association tree
ADD_EXECUTABLE(test_tree test_tree.cpp ../association_node.cpp ../association_tree.cpp ../association_solver.cpp)

