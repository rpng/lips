% Clear and close everything
clear all;
close all;

% Add paths needed
addpath('functions/matGeom/geom3d/')
addpath('functions/lips/')


% Load our data from file
path = '../input/floorplan_spencer_small.txt';
planes2d = load_2dplanedata(path);

% Convert to polygons with a height of 8ft
height = 8;
planes3d = planes2dtopolygons3d(planes2d, height);


% Plot the polyons (uses geom3d function)
figure;
for ii=1:size(planes3d,2)
	drawPolygon3d(planes3d{ii}(:,1),planes3d{ii}(:,2),planes3d{ii}(:,3),'b');
    hold on;
end
drawCoordinates3d([0,0,0],[0,0,0],10)
axis equal
xlabel('x-direction (ft)')
ylabel('y-direction (ft)')
zlabel('z-direction (ft)')



















