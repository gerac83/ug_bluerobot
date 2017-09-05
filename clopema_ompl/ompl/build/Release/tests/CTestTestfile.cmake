# CMake generated Testfile for 
# Source directory: /home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/tests
# Build directory: /home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_heap "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_heap")
add_test(test_grid "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_grid")
add_test(test_gridb "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_gridb")
add_test(test_nearestneighbors "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_nearestneighbors")
add_test(test_pdf "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_pdf")
add_test(test_random "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_random")
add_test(test_machine_specs "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_machine_specs")
add_test(test_state_operations "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_state_operations")
add_test(test_state_spaces "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_state_spaces")
add_test(test_state_storage "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_state_storage")
add_test(test_ptc "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_ptc")
add_test(test_planner_data "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_planner_data")
add_test(test_2denvs_geometric "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_2denvs_geometric")
add_test(test_2dmap_geometric_simple "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_2dmap_geometric_simple")
add_test(test_2dmap_ik "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_2dmap_ik")
add_test(test_2dcircles_opt_geometric "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_2dcircles_opt_geometric")
add_test(test_2dmap_control "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_2dmap_control")
add_test(test_planner_data_control "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/bin/test_planner_data_control")
subdirs(regression_tests)
