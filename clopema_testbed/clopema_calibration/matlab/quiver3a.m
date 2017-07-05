function quiver3a(A,B,S,linespec)
%QUIVER3A
%
% SYNOPIS
%   quiver3a(A,B,S = 1, linespec = 'r')
%
% DESCRIPTION
%   The following calls are equivalent:
%       quiver3a(A,B)
%       quiver3(A(1,:), A(2,:), A(3,:), S*(B(1,:) - A(1,:), S*(B(2,:) - A(2,:), S*(B(3,:) - A(3,:), 0))
%

% Libor Wagner on April 26, 2013

if nargin <= 2 | isempty(S), S = 1; end;
if nargin <= 3 | isempty(linespec), linespec = 'r'; end;
quiver3(A(1,:), A(2,:), A(3,:), S*(B(1,:) - A(1,:)),S*(B(2,:) - A(2,:)),S*(B(3,:) - A(3,:)),0,linespec);

end
