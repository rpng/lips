function [ rays ] = gencirclerays( pos_LIDARinG, rpy_LIDARtoG, azimuth, resolution, mag )
%GENCIRCLERAYS Generates 360 deg of rays given the two angles
%   We need the origin position, the azimuth angle from the horizontial,
%   the resolution of the lidar horizontially, and then the magnitude we
%   want our direction to be (mostly used for visulization

% Our final rays object
rays = {};

% Divide our 360 degrees by the desired resolution
numbeams = floor(360/resolution);
resolution = 360/numbeams;

% Debug info
fprintf('RAY: Using %d beams (%.2f resolution)\n',numbeams,resolution);

% From here, we can start from zero and go till 360 and generate our rays
for deg=0:resolution:360
    % Our ray is a 1x6
    ray = zeros(1,6);
    % First three are the origin of the ray
    ray(1,1) = pos_LIDARinG(1,1);
    ray(1,2) = pos_LIDARinG(1,2);
    ray(1,3) = pos_LIDARinG(1,3);
    % Next three are the "direction" of the ray
    % Convert spherical coordinates to our x,y,z cartesian
    ray(1,4) = mag*sind(90-azimuth)*cosd(deg);
    ray(1,5) = mag*sind(90-azimuth)*sind(deg);
    ray(1,6) = mag*cosd(90-azimuth);
    % Now those rays are in the local LIDAR frame
    % So lets rotate them into the global (note rotx,y,z requires degrees)
    rot_LIDARtoG = rotz(180/pi*rpy_LIDARtoG(1,3))*roty(180/pi*rpy_LIDARtoG(1,2))*rotx(180/pi*rpy_LIDARtoG(1,1));
    ray(1,4:6) = (rot_LIDARtoG*ray(1,4:6)')';
    % Append to our rays
    rays{end+1} = ray;
end



end

