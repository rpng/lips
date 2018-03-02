function [ planes3d_PinG ] = addhorzplane( planes2d_PinG, planes3d_PinG, height )
%ADDHORZPLANE Will add the ground plane as a polygon
%   Will find the min and max locations in the x,y plane. From there it
%   will add a plane that is at a zero in the z-direction.

% Our recorded min and max valud
xmax = -Inf;
xmin = Inf;
ymax = -Inf;
ymin = Inf;

% Loop through each plane to find the min and max x,y
for ii=1:size(planes2d_PinG,1)
    % x-direction
    xmax = max(xmax,planes2d_PinG(ii,1));
    xmax = max(xmax,planes2d_PinG(ii,3));
    xmin = min(xmin,planes2d_PinG(ii,1));
    xmin = min(xmin,planes2d_PinG(ii,3));
    % y-direction
    ymax = max(ymax,planes2d_PinG(ii,2));
    ymax = max(ymax,planes2d_PinG(ii,4));
    ymin = min(ymin,planes2d_PinG(ii,2));
    ymin = min(ymin,planes2d_PinG(ii,4));
end

% check to make sure that we do not have the same values
if xmax == xmin || ymax == ymin
   error('DATA: INVALID XMAX XMIN YMAX YMIN VALUES!!!!!!');
end


% check to make sure that we do not have the same values
if abs(xmax) == Inf || abs(xmin) == Inf || abs(xmax) == Inf || abs(ymin) == Inf
   error('DATA: INF MIN MAX VALUES!!!!!!!');
end


% debug info
fprintf('DATA: Added plane xmin/max = (%.2f,%.2f) ymin/max = (%.2f,%.2f) z = %.2f\n',xmin,xmax,ymin,ymax,height)

% else we are good to add this plane
poly = zeros(4,3);
poly(1,:) = [xmin ymin height];
poly(2,:) = [xmax ymin height];
poly(3,:) = [xmax ymax height];
poly(4,:) = [xmin ymax height];
planes3d_PinG{end+1} = poly;



end

