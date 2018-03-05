% Clear and close everything
clear all;
close all;

% Add paths needed
addpath('functions/matGeom/polygons2d/')
addpath('functions/matGeom/geom3d/')
addpath('functions/lips/')


% Load our data from file
path = '../input/floorplan_spencer_small.txt';
planes2d = load_2dplanedata(path);

% Convert to polygons with a height of 8ft
height = 8;
planes3d = planes2dtopolygons3d(planes2d, height);


% Next up, set our position to where we want to shoot a ray out
pos = [12 12 4]; % meters
ypr = [0 0 0]; % rad
resolution = 5; % deg
distance = 10; % meter (just for viz)
zenith = 0; % deg


% Generate our rays
rays1 = gencirclerays(pos, ypr, zenith, resolution, distance);

% Interect with our planes/polygons
%[hits, polyids, rayids] = intersectrayspolys(rays1, planes3d);
[hits, polyids, rayids] = kevinsPlaneIntersection(rays1, planes3d);


% Plot the polyons (uses geom3d function)
figure;
for ii=1:size(planes3d,2)
	drawPolygon3d(planes3d{ii}(:,1),planes3d{ii}(:,2),planes3d{ii}(:,3),'b');
    hold on;
end
for ii=1:size(rays1,2)
    x = [rays1{ii}(1,1) rays1{ii}(1,1)+rays1{ii}(1,4)];
    y = [rays1{ii}(1,2) rays1{ii}(1,2)+rays1{ii}(1,5)];
    z = [rays1{ii}(1,3) rays1{ii}(1,3)+rays1{ii}(1,6)];
    drawPolyline3d(x,y,z,'-k')
    hold on;
end
% for ii=1:size(rays2,2)
%     drawPolyline3d([rays2{ii}(1,1) rays2{ii}(1,4)],[rays2{ii}(1,2) rays2{ii}(1,5)],[rays2{ii}(1,3) rays2{ii}(1,6)],'g')
%     hold on;
% end
for ii=1:size(hits,1)
    drawPoint3d(hits(ii,1),hits(ii,2),hits(ii,3),'.-k');
    hold on;
end
drawCoordinates3d(pos, ypr, 5)
axis equal
xlabel('x-direction (ft)')
ylabel('y-direction (ft)')
zlabel('z-direction (ft)')



















