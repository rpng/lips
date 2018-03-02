% Clear and close everything
clear all;
%close all;

% Add paths needed
addpath('functions/lips/')
addpath('functions/cursorloc/')

% Load our data from file
path = '../input/floorplan_2d_spencer_v2.txt';
data = load_2dplanedata(path);

% Plot these lines
fh = figure(1);
clf(fh)
for ii=1:size(data,1)
    x = [data(ii,1) data(ii,3)];
    y = [data(ii,2) data(ii,4)];
	plot(x,y, '-r');
    hold on;
end
drawCoordinates3d([0,0,0],[0,0,0],10)
axis equal;
xlabel('x-direction (ft)')
ylabel('y-direction (ft)')

set(fh, 'WindowButtonMotionFcn', @(obj, event)cursorLocation(obj, event, 'BottomLeft', ' X: %.3f\n Y: %.3f', 'k'))














