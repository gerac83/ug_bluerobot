% Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
%
% Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
% Institute:   Czech Technical University in Prague
% Created on:  Dec 3, 2013

% Options
dataset = '../data/perspective';
scale = 300;

% Read info file
format = '%s %f %f %f %f %f %f %f %f';
f = fopen([dataset '/info.txt']);
C = textscan(f, format, 'delimiter', ',');
fclose(f);

N = size(C{1}, 1);
names = C{1};
corners = cat(2, C{2:end});

% Mapping points
b = [0,0,1,1;1,0,0,1] * scale;
% Read images and perform transformation
for i = 1:N,
    im = im2double(imread([dataset '/' names{i}]));
    a = [corners(i,1:4); corners(i,5:8)];

    % Get perspective transform from a to b
    T = get_perspective_transform(a,b);

    % Transform the image
    im2 = apply_projective_transform(im, T, 0:300, 0:300);
    p = [150, 150]';

    % And transform it back
    im3 = apply_projective_transform(im2, T\diag([1,1,1]), 1:640, 1:480);

    % Transform point p found in the transformed image back to the original
    % coordinates
    a_ = unhom(T\hom(p));

    % Show result
    subplot(1,3,1), imshow(im);
    subplot(1,3,2), imshow(im2); hold on
    plot(p(1,:), p(2,:), 'xr'); hold off
    subplot(1,3,3), imshow(im3); hold on


    plot(a_(1,:), a_(2,:), 'xr'); hold off
    drawnow()
    pause(0.2)

end

