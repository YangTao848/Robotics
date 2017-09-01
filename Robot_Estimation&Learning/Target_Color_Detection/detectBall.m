% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

I=double(I);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
 mu = [145.6488,142.3374,65.2150];
 sig = [2.823727067030126e+02,1.700636580311599e+02,-2.962629855466670e+02;
     1.700636580311599e+02,1.549661427730383e+02,-2.102779432686208e+02;
     -2.962629855466670e+02,-2.102779432686208e+02,4.470626009837190e+02];
 thre = 6e-6;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
dI = double(I);
row = size(dI, 1); 
col = size(dI, 2);
for i = 1:3
    dI(:,:,i) = dI(:,:,i) - mu(i);
end
dI = reshape(dI, row*col, 3);
dI = exp(-0.5 * sum(dI * inv(sig) .* dI, 2)) ./ (2 * pi)^1.5 ./ det(sig)^0.5;
dI = reshape(dI, row, col);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
dI = dI > thre;
segI = false(size(dI));
CC = bwconncomp(dI);
numPixels = cellfun(@numel,CC.PixelIdxList);
[~,idx] = max(numPixels);
segI(CC.PixelIdxList{idx}) = true; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
