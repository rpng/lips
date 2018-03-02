function [ planes3d ] = planes2dtopolygons3d( planes2d, height )
%2DTO3DPLANES Takes in 2D lines, and returns polygons based on the height
%   Given a 2D set of lines, we will create a set of 3D points that define
%   a polygon. This polygon is a plane restricted to a specific area.

% Will return a set of cells
planes3d = {};

% Loop through each 2D line and make its polygon
% We create 4 points (bottom left, bottom right, top right, top left)
for ii=1:size(planes2d,1)
    % Create the poly
    poly = zeros(4,3);
    poly(1,:) = [planes2d(ii,1) planes2d(ii,2) 0];
    poly(2,:) = [planes2d(ii,3) planes2d(ii,4) 0];
    poly(3,:) = [planes2d(ii,3) planes2d(ii,4) height];
    poly(4,:) = [planes2d(ii,1) planes2d(ii,2) height];
    %poly(5,:) = [planes2d(ii,1) planes2d(ii,2) 0];
    % Append to our cell list
    planes3d{end+1} = poly;
end


% Debug info
fprintf('DATA: Converted %d 2D planes\n',size(planes3d,2))


end

