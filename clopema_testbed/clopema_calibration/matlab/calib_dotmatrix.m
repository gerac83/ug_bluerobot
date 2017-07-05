function res = calib_dotmatrix(images, conf)
%CALIB_DOTMATRIX
%
% SYNOPSIS
%   res = calib_dotmatrix(images, conf)
%
% INPUT (i - index of the robot pose)
%   images              Cell array of image paths.
%   conf = struct()     Configuration structure see dotmatrix.m for details.
%
% OUTPUT
%   res = struct()
%   res(i).img_path
%   res(i).X
%   res(i).x
%   res(i).ind
%   res(i).valid

% Libor Wagner on May  2, 2013

% Detect dots
N = numel(images);
res = struct('img_path', cell(1,N), 'X',cell(1,N),'x',cell(1,N),'valid',cell(1,N));
for i = 1:N
    try
        res(i).img_path = images{i};
        D = dotmatrix(images{i}, conf);
    catch e,
        res(i).valid = 0;
        fprintf(1,'Unable to detect dots in %s, error: %s\n', images{i}, e.message);
        continue;
    end

    res(i).valid = 1;
    res(i).X = [D(:,[3,4]), zeros(size(D,1),1)]';
    res(i).x = D(:,[6,5])';
    res(i).ind = D(:,[1,2])';
end


end
