function X = hom(x)
%HOM Convert column vectors to homogenous.
%
% SYNOPSIS:
%   X = hom(x)

% Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
%
% Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
% Institute:   Czech Technical University in Prague
% Created on:  Dec 2, 2013

X = [x; ones(1, size(x,2))];
end
