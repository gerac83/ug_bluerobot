function im2 = apply_projective_transform(im, tform, xrange, yrange)
%APPLY_PROJECTIVE_TRANSFORM

% Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
%
% Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
% Institute:   Czech Technical University in Prague
% Created on:  Dec 3, 2013

[X,Y] = meshgrid(1:size(im,2), 1:size(im,1));
[Xq,Yq] = meshgrid(xrange, yrange);

q = unhom(tform\hom([Xq(:)'; Yq(:)']));
Xq(:) = q(1,:);
Yq(:) = q(2,:);


im2 = zeros(size(Xq, 1), size(Xq,2), 3);
im2(:,:,1) = interp2(X,Y,im(:,:,1), Xq, Yq);
im2(:,:,2) = interp2(X,Y,im(:,:,2), Xq, Yq);
im2(:,:,3) = interp2(X,Y,im(:,:,3), Xq, Yq);
end

