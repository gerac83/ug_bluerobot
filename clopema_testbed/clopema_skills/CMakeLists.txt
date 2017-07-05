cmake_minimum_required(VERSION 2.8.3)
project(clopema_skills)
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs sensor_msgs geometry_msgs message_generation roscpp rospy std_msgs)

add_action_files(
  DIRECTORY action
  FILES FoldingStria.action
        PickUp.action
        Unfold.action
        RecognizeGarment.action
	    CerthClassifier.action
	    CapturingRH.action
	    SaccadeVerge.action
	    PointCloudRH.action
	    PCfromImgsRH.action
	    GraspingRH.action
	    GraspingFoveatedRH.action
	    FlatteningRH.action
	    Grasping.action
	    PutOverEdge.action
        GPiling.action
        PileInfo.action
	    PutIntoSortedHeaps.action
	    RecognitionRH.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs sensor_msgs geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs sensor_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
