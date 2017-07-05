OMPL dependencies:
* [Boost](http://www.boost.org) (version 1.44 or higher)
* [CMake](http://www.cmake.org) (version 2.8.2 or higher)

#######################################
# How to set up the modified planners #
#######################################

1) Download this repository into "/home/<your_user_name>/ompl_clopema" and navigate your terminal to the ompl_clopema folder

2) Compile OMPL with the following commands (http://moveit.ros.org/wiki/OMPL/Add_New_Planner#Install_MoveIt.21_from_Source):

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    make -j 4 # replace "4" with the number of cores on your machine

3) Download the clopema_planners_benchmarking repository into the CloPeMa workspace

4) Copy (and overwrite) ompl_planning.yaml from clopema_planners_benchmarking/src/related/OMPL modifications to [clopema_testbed package]/clopema_moveit_config/config 
		(modify the list of planners if you want to include some others or alter some parameters)

5) Compile the CloPeMa workspace

6) Roslaunch the robot and hope for the best