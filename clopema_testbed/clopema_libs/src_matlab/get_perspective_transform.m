function T = get_perspective_transform(x1, x2)
%GET_PERSPECTIVE_TRANSFORM from four corespondences.
%
% SYNOPSIS:
%   T = get_projective_transform(x1, x2)
%
% INPUT:
%   x1,x2       [2x4] Coresponding points in a plane
%
% OUTPUT:
%   T           [3x3] Transformation matrix so that h(x2) = T * h(x1) where h(x)
%               are a homegenous cordinates of x.

% Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
%
% Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
% Institute:   Czech Technical University in Prague
% Created on:  Dec 2, 2013

% Construct equations
A = zeros(9,9);
A(1:4,:) = [x1' ones(4,1) zeros(4,3) -x2([1,1],:)'.*x1' -x2(1,:)'];
A(5:8,:) = [zeros(4,3) x1' ones(4,1) -x2([2,2],:)'.*x1' -x2(2,:)'];

% Solve Ax = 0
[~,~,V] = svd(A);
T = reshape(V(:,end), 3,3)';

% Normalize matrix
T = T / T(3,3);

end

