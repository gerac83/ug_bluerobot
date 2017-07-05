function export_link(path, link, T)
%EXPORT_LINK Export calibration of a certain link.
%
% SYNOPSIS
%   export_link(path, link, T)
%
% INPUT
%   path    Path to exported file
%   link    Link name
%   T       Transformation from parent to the link

% Libor Wagner on September 24, 2013

% Open file for writing or use the given stream directly
if ischar(path)
    f = fopen(path,'w');
    if f == -1,
        error('Unable to open file %s', path);
    end
else
    f = path;
end

% Write type, link and the transformation
fprintf(f, '%d\n', 2);
fprintf(f, '%s\n', link);
for r = 1:size(T,1)
    for c = 1:size(T,2)
        fprintf(f, ' % 0.17f', T(r,c));
    end
    fprintf(f,'\n');
end

% Close file is openned by us
if ischar(path)
    fclose(f);
end

end
