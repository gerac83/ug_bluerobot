function [res] = calib_camera(points, conf)
%CALIB_CAMERA
%
% SYNOPSIS
%   [res] = calib_camera(conf,camera)
%
% INPUT (i - index of robot pose, M - number of points)
%   points = struct()
%   points(i).X
%   points(i).x
%   points(i).valid
%
%   conf = struct()
%   conf.output_tpl         Template for intermediate results
%   conf.scale              Scale of the X to robot units
%
% OUTPUT
%   res = struct()
%   res.K           Camera matrix (3,3)
%   res.dist        Distortion coefficients (1x5)

% Libor Wagner on April 30, 2013

% Get input size
N = length(points);

active_images = cat(2,points.valid)

% Fix input
for i = 1:N
    if active_images(i),
        xi = points(i).x;
        Xi = points(i).X;
        eval(sprintf('x_%d = xi(:,~any(isnan(xi)));', i));
        eval(sprintf('X_%d = Xi(:,~any(isnan(xi)));', i));
    else
        eval(sprintf('x_%d = NaN(1);', i));
        eval(sprintf('X_%d = NaN(1);', i));
    end
end

n_ima = N;
go_calib_optim_iter; % run the calibration

res = struct();
res.K = [diag(fc), cc; 0, 0, 1];
res.dist = kc';

for i = 1:N
    if active_images(i),
        eval(sprintf('res.pose(%d).t = Tc_%d;', i, i));
        eval(sprintf('res.pose(%d).R = Rc_%d;', i, i));
    end
end


dX = conf.cc_scale;
dY = conf.cc_scale;
[pathstr,~,~] = fileparts(conf.output_tpl);
if exist(pathstr, 'dir') ~= 7,
    mkdir(pathstr);
end

save_name =  [conf.output_tpl '/camera_calib'];
saving_calib; %save output

end
