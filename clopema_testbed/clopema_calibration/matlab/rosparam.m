function value = rosparam(name)
%ROSPARAM Get ROS parameter from parameter server.
%
% SYNOPSIS
%   value = rosparam(name)
%
% INPUT
%   name  Parameter name
%
% OUTPUT
%   value Parameter value
%
% NOTE
%   This function ends with error when the rosparam ends with error.

% Libor Wagner on May 10, 2013

call = ['rosparam get ' name];
[status,cmdout] = system(call);

if status ~= 0,
    error(cmdout);
else
    % Remove the new line character.
    value = cmdout(1:end-1);
end

end


