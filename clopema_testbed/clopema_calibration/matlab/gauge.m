function gauge(path, conf)
%GAUGE Generate gauge file.
%
% SYNOPSIS
%   gauge(path, conf)
%
% INPUT
%   path            Path to the new gauge file.
%   conf.name       Name of the gauge i.e. 30x30
%   conf.cols       List of columns i.e. -15:14
%   conf.rows       List of rows i.e. -15:14
%   conf.colstep    Distance between columns in meters i.e. 0.029
%   conf.rowstep    Distance between rows in meters i.e. 0.029
%   conf.keyr       Radius of the key dots in meters i.e. 0.002
%   conf.dotr       Radius of the normal dots in meters i.e. 0.001


if ischar(path)
    f = fopen(path, 'w');
    if f == -1,
        error('Unable to open file %s', path);
    end
else
    f = path;
end

fprintf(f,'\n');
fprintf(f, '; Gauge for optics calibraton is a set of dots organized into grid.\n');
fprintf(f, '; The grid has %d columns and %d rows with 3 dots defining coordinate system.\n', numel(conf.cols), numel(conf.rows));
fprintf(f, '; Key dots are detected by different diameter.\n');
fprintf(f,'\n');
fprintf(f,'[Header]\n');
fprintf(f,'\n');
fprintf(f,'Type     = DotMatrix\n');
fprintf(f,'ID       = %s\n', conf.name);
fprintf(f,'\n');
fprintf(f,'[Data]\n');
for c = conf.cols
    for r = conf.rows
        if (c == 2 && r == 0) || (c == 0 && r == 0) || (c == 0 && r == 1)
            fprintf(f,'%s %s %s\n', format(c*conf.colstep), format(r*conf.rowstep), format(conf.keyr));
        else
            fprintf(f,'%s %s %s\n', format(c*conf.colstep), format(r*conf.rowstep), format(conf.dotr));
        end
    end
end

if ischar(path)
    fclose(f);
end
end

function s = format(a)
str = sprintf('% .7e', a);
s = [str(1:end-2) '0' str(end-1:end)];
end


