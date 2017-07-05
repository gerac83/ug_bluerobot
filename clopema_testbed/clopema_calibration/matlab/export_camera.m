function export_camera(camera, path)
%EXPORT_CAMERA Export camera info to a file.
%
% SYNOPSIS
%   export_camera(camera, paht)
%
% INPUT
%   camera  Structure representing camera.
%   path    Output file path.

% Libor Wagner on May  6, 2013

% Open file
if ischar(path),
    f = fopen(path,'w');
    if f == -1,
        error('Unable to open file %s', path);
    end
else
    f = 1;
end

fprintf(f, 'image_width: %d\n', camera.width);
fprintf(f, 'image_height: %d\n', camera.height);
if isfield(camera, 'name')
    fprintf(f, 'camera_name: %s\n', camera.name);
else
    fprintf(f, 'camera_name: %s\n', 'camera');
end
fprintf(f, 'camera_matrix:\n');
fprintf(f, '    rows: 3\n');
fprintf(f, '    cols: 3\n');
fprintf(f, '    data: [%f, %f, %f, %f, %f, %f, %f, %f, %f]\n', camera.K');
fprintf(f, 'distortion_model: plumb_bob\n');
fprintf(f, 'distortion_coefficients:\n');
fprintf(f, '    rows: 1\n');
fprintf(f, '    cols: 5\n');
fprintf(f, '    data: [%f, %f, %f, %f, %f]\n', camera.dist);
fprintf(f, 'rectification_matrix:\n');
fprintf(f, '    rows: 3\n');
fprintf(f, '    cols: 3\n');
fprintf(f, '    data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n');
fprintf(f, 'projection_matrix:\n');
fprintf(f, '    rows: 3\n');
fprintf(f, '    cols: 4\n');
fprintf(f, '    data: [%f, %f, %f, 0, %f, %f, %f, 0, %f, %f, %f, 0]\n', camera.K');

% Close file
if ischar(path),
    fclose(f);
end

end
