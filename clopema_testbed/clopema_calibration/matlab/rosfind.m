function path = rosfind(package)
%ROSFIND
%
% SYNOPSIS
%   path = rosfind(package)
%
% INPUT
%   package     Package name
%
% OUTPUT
%   path        Path to the package
%

% Libor Wagner on April 30, 2013

    [status, result] = system(['rospack find ' package]);
    if status == 0,
        path = result(1:end-1);
    else
        path = '';
    end
end
