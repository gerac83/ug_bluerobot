function robot_stop(force)
    rosClopemaCallService('/joint_trajectory_action/set_power_off',struct(''));
end
