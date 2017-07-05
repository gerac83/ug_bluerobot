function [pose] = calib_dir(conf)
%CALIB_DIR Read content of the calibration dir.
%
% SYNOPSIS
%   poses = calib_dir(conf)
%
% INPUT
%   conf = struct()
%   conf.input_dir  Directory that should be searched for calibration files.
%   conf.input_ext  Cell array of possible extensions.
%   conf.verbose    0/1 verbosity flag
%
% OUTPUT
%   Output is a structure array that contain paths for all files that follow
%   format described bellow. Each path can than be accesed through as
%
%               pose(i).type
%
%   Where i is the pose number and type is what the data type.
%
% FORMAT
%   Files in the calibration dir must follow the followign format. Each file
%   should start with pose number then followed by underscore (_) and type.

% Libor Wagner on May  2, 2013

if ~isfield(conf, 'verbose'), conf.verbose = 0; end

% Prepare output structure
pose = struct();

dl = dir(conf.input_dir);
for i = 1:length(dl),
    str = strsplit(dl(i).name, {'_','.'});
    if length(str) == 3,
        num = str2num(str{1});
        type = str{2};
        ext = str{3};

        if any(strcmp(ext, conf.input_ext)),
            pose(num).(type) = [conf.input_dir '/' dl(i).name];
            if conf.verbose == 1, fprintf(1,'Adding %s\n', dl(i).name); end
        end
    else
        if conf.verbose == 1, fprintf(1,'Skipping %s\n', dl(i).name); end
    end
end

empty = zeros(1,length(pose));
for i = 1:length(pose)
    empty(i) = isempty(pose(i).RGB);
end
pose = pose(~empty);

end
