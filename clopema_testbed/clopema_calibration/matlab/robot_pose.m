function robot_pose(pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w)
% GO_POSE_R1 Move robot 1 to desired position

    % TODO: Check motoros
    % Check parameters
    if nargin==1 && (length(pos_x) > 1)
        pos_y = pos_x(2);
        pos_z = pos_x(3);
        ori_x = pos_x(4);
        ori_y = pos_x(5);
        ori_z = pos_x(6);
        ori_w = pos_x(7);
        pos_x = pos_x(1);
    end

    % Request robot current robot state
    robotState = rosClopemaCallService('/environment_server/get_robot_state',struct(''));
    if ~isstruct(robotState)
        fprintf('Can''t read robot state.\n');
        return;
    end

    % Create pose request
    req.motion_plan_req.allowed_planning_time.secs = 5;
    req.motion_plan_req.group_name = 'r2_arm';
    req.motion_plan_req.start_state = robotState.robot_state;

    % Specify pose position
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
    pos.position.x = pos_x;
    pos.position.y = pos_y;
    pos.position.z = pos_z;

    % Specify pose orientation
    ori.header.frame_id = 'base_link';
    ori.link_name = pos.link_name;
    ori.absolute_pitch_tolerance = 0.05;
    ori.absolute_roll_tolerance = 0.05;
    ori.absolute_yaw_tolerance = 0.05;
    ori.weight = 1.0;
    ori.orientation.x = ori_x;
    ori.orientation.y = ori_y;
    ori.orientation.z = ori_z;
    ori.orientation.w = ori_w;

    % Finish request
    req.motion_plan_req.goal_constraints.position_constraints{1} = pos;
    req.motion_plan_req.goal_constraints.orientation_constraints{1} = ori;

    % Plan trajectory
    res = rosClopemaCallService('clopema_planner/plan', req);

    % Check plan and execute if possible
    if (res.error_code.val == 1)
        goal = rosClopemaCreateMsg('control_msgs/FollowJointTrajectoryGoal');
        goal.trajectory = res.joint_trajectory;
        result = rosClopemaSendGoal('control_msgs/FollowJointTrajectory', '/clopema_controller/follow_joint_trajectory', goal, 30.0);
    else
        fprintf('Failed to plan trajectory, error code: %d\n', res.error_code.val);
    end
end
