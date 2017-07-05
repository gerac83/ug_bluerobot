function [D, valid] = dotmatrix(image, conf)
%DOTMATRIX Matlab wrapper for dotmatrix binary.
%
% SYNOPSIS
%   [D, valid] = dotmatrix(image, conf)
%
% INPUT
%   image              Input image
%   conf = struct()
%   conf.dm_binary     Path to dotmatrix binary.
%   conf.dm_algorithm  Path to file containing algorithm chain.
%   conf.dm_gauge      Path to gauge file.
%   conf.ld_path       Path to directory containing libdotmatrix.
%   conf.show          1/0 show the result in figure (default: 0).
%   conf.verbose       1/0 show the result in figure (default: 0).
%
% OUTPUT
%   D               Dotmatrix output
%   D(:,1:2)        Dot index in the grid pattern.
%   D(:,3:4)        Dot position in the gauge
%   D(:,5:6)        Dot position in the image
%   D(:,7)          Dot area
%   valid           Mask of valid points

% Libor Wagner on September 20, 2013


% Check input
if nargin < 1,
    error('At least one argument is required.')
end

if ~isfield(conf,'show'), conf.show = 0; end
if ~isfield(conf,'verbose'), conf.verbose = 0; end

% Dotmatrix call
LD_PATH = ['LD_LIBRARY_PATH=' conf.ld_path];
CALL = [LD_PATH ' ' conf.dm_binary ' ' conf.dm_algorithm ' ' conf.dm_gauge ' ' image];

% Show call if requested verbose output
if conf.verbose > 1, disp(['call="' CALL '"']); end

% Call dotmatrix
[status, result] = system(CALL);

% Process the result
if status == 0 && ~isempty(result),
    C = textscan(result, '%d %d %f %f %f %f %f', 'delimiter', ',', 'CollectOutput', 1);
    N = size(C{1},1);


    D = zeros(N, 7);
    D(:,1:2) = C{1};
    D(:,3:7) = C{2};

    % Get valid points
    valid = ~(any(isnan(D),2));

    % Prin info if requested verbose output
    if conf.verbose, fprintf(1,'Detected %d dots in %s\n', sum(valid), image); end
else
    disp(result);
    error('No points were found, status = %d.', status);
end

% Show resul if requested
if conf.show
    figure();
    imshow(image);
    hold on;
    plot(D(valid,6)+1, D(valid,5)+1, 'rx');
end

end
