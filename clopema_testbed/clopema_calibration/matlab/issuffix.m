function c = issuffix(a, b)
%ISSUFFIX Is a suffix of b?
%
% SYNOPSIS
%   c = issuffix(a, b)

% Libor Wagner on April 22, 2013

    if length(a) > length(b)
        c = 0;
    else
        b_ = b(length(b) - length(a) + 1:end);
        c  = strcmp(a,b_);
    end

end
