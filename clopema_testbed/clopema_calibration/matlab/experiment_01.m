%% Init (addpath)
clc;clear all;close all;
[voopRes voopT] = system('echo $ROS_WORKSPACE');
addpath(genpath([strcat(voopT) '/clopema_matlab/matlab/']));
clear voopRes voopT

%% Specify request for trajectory and plan
%ee_pose = [0.5 -1.0 0.8 1 0 0 0];   %xyz quaternion
go_home = true;
start_position = true;
for x = -0.20:0.1:0.20
    for y = -0.20:0.1:0.20
        ee_pose = [0.7+x -1.1+y 0.8 1 0 0 0];
        if go_home
            ee_pose = [0.5253 -0.82 1.6977 0.701 -0.701 -0.092 0.092];
        end
        clear req;
        req.motion_plan_req.allowed_planning_time.secs = 5;
        req.motion_plan_req.group_name = 'r2_arm';
        robotState = rosClopemaCallService('/environment_server/get_robot_state',struct(''));
        if ~isstruct(robotState)
            fprintf('Can''t read robot state.\n');
            return
        end
        req.motion_plan_req.start_state = robotState.robot_state;
        % Specify pose position and orientation
        pos.header.frame_id = 'base_link';
        pos.link_name = 'r2_tip_link';
        pos.constraint_region_orientation.x = 0;
        pos.constraint_region_orientation.y = 0;
        pos.constraint_region_orientation.z = 0;
        pos.constraint_region_orientation.w = 1;
        pos.constraint_region_shape.type = 1; %box shape
        pos.constraint_region_shape.dimensions{1} = 0.05;
        pos.constraint_region_shape.dimensions{2} = 0.05;
        pos.constraint_region_shape.dimensions{3} = 0.05;
        pos.weight = 1.0;
        pos.position.x = ee_pose(1);
        pos.position.y = ee_pose(2);
        pos.position.z = ee_pose(3);

        ori.header.frame_id = 'base_link';
        ori.link_name = pos.link_name;
        ori.absolute_pitch_tolerance = 0.05;
        ori.absolute_roll_tolerance = 0.05;
        ori.absolute_yaw_tolerance = 0.05;
        ori.weight = 1.0;
        ori.orientation.x = ee_pose(4);
        ori.orientation.y = ee_pose(5);
        ori.orientation.z = ee_pose(6);
        ori.orientation.w = ee_pose(7);

        req.motion_plan_req.goal_constraints.position_constraints{1} = pos;
        req.motion_plan_req.goal_constraints.orientation_constraints{1} = ori;


        if ~start_position
            ori.link_name = 'r2_link_6';
            ori.absolute_pitch_tolerance = 0.5;
            ori.absolute_roll_tolerance = 0.5;
            ori.absolute_yaw_tolerance = 0.5;
            ori.orientation.x = 0;
            ori.orientation.y = 0;
            ori.orientation.z = -0.78;
            ori.orientation.w = 0.61;
            req.motion_plan_req.path_constraints.orientation_constraints{1} = ori;
        end
        start_position = false;

        res = rosClopemaCallService('clopema_planner/plan',req);

        if(res.error_code.val == 1)
            %% Execute trajectory
            goal = rosClopemaCreateMsg('control_msgs/FollowJointTrajectoryGoal');
            fprintf('Sending goal %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n',ee_pose);
            goal.trajectory = res.joint_trajectory;
            result = rosClopemaSendGoal('control_msgs/FollowJointTrajectory', '/clopema_controller/follow_joint_trajectory', goal, 30.0);
            pause(10)
        else
            fprintf('Error code: %d\n', res.error_code.val);
        end    
        if go_home
            break;
        end
    end
    if go_home
        break;
    end
end
%% Stop servo
rosClopemaCallService('/joint_trajectory_action/set_power_off',struct(''));
