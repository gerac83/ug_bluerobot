function x = unhom(X)
%UNHOM Convert from homegenous to normal
%
% SYNOPSIS:
%   x = unhom(X)

% Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
%
% Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
% Institute:   Czech Technical University in Prague
% Created on:  Dec 2, 2013

x = X(1:(end-1),:) ./ X(ones(1,size(X,1)-1)*end, :);
end
