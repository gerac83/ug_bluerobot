function res = calib_stereo(conf, left, right)
%CALIB_STEREO Stereo calibration wrapper for Matlab Callibration Toolbox
%
% SYNOPSIS
%   res = calib_stereo(poses, left, right, scale)
%
% INPUT
%   conf.camera_[left|right]
%   conf.handeye_[left|right]
%   left
%   right
%
% OUTPUT
%   res 
%

% Libor Wagner on May  2, 2013

% Fields
FIELD_left_cam = sprintf('camera_%s', left);
FIELD_righ_cam = sprintf('camera_%s', right);

calib_file_name_left = conf.(FIELD_left_cam).calib;
calib_file_name_right = conf.(FIELD_righ_cam).calib;
recompute_intrinsic_left = 0; recompute_intrinsic_righ = 0;

load_stereo_calib_files2;

save_name = [conf.output '/'  'calib_stereo.mat'];
calib_stereo_auto2;

res = struct();
res.calib = save_name;
res.t = T;
res.R = rodrigues(om);
res.T = [res.R, res.t; 0 0 0 1];

end
