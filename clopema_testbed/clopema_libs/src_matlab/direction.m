% Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
%
% Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
% Institute:   Czech Technical University in Prague
% Created on:  Nov 26, 2013

function aa = direction(A, B, show)

% Fix input
if nargin < 3, show = 0; end

if show
    figure(); hold on
    plot3(A(1), A(2), A(3), 'xr');
    plot3(B(1), B(2), B(3), 'xr');
end

% Vector from A to B
Z = B - A;

% Vector perpendicular to Z and baselink Z
X = cross(Z, [0,0,1]);

if all(X==0),
    X = [1,0,0];
end

% Vector perpendicular to new Z and new X
Y = cross(Z,X);

% Normalize all
X = X / sqrt(sum(X.^2));
Y = Y / sqrt(sum(Y.^2));
Z = Z / sqrt(sum(Z.^2));


if show
    quiver3(A(1), A(2), A(3),X(1), X(2), X(3), '-r');
    quiver3(A(1), A(2), A(3),Y(1), Y(2), Y(3), '-g');
    quiver3(A(1), A(2), A(3),Z(1), Z(2), Z(3), '-b');
end


R = [X' Y' Z']

det(R)
inv(R)


end

