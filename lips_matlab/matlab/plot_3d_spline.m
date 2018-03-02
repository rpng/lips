% Clear and close everything
clear all;
close all;

% Add paths needed
addpath('functions/matGeom/polygons2d/')
addpath('functions/matGeom/geom3d/')
addpath('functions/lips/')

%% CONFIG INFORMATION IS ALL IN HERE

pathplanes = '../input/floorplan_2d_spencer_v2.txt';
pathpath = '../input/path_spencer_01_v2.txt';

planeheight = 3; %meters

imurate = 200; %hz
lidarrate = 0.5; %hz

totalruntime = 360; %seconds



%% ROBOT TRAJECTORY LOADING AND PROCESSING
% Load our data from file
data = csvread(pathpath,1,0);
%pathtime = linspace(0,totalruntime,size(path_IMUinG,1));
pathtime = totalruntime*data(:,1)/max(data(:,1));
path_IMUinG = data(:,2:end);

% NOTE: Convert from feet poses to meters
path_IMUinG(:,1:3) = 0.3048.*path_IMUinG(:,1:3);

% Convert from the degrees to radians
path_IMUinG(:,4:6) = pi/180.*path_IMUinG(:,4:6);

% Create a spline for each dimension x,y,z and roll,pitch,yaw
% This will be used to evaluate the derivative to give us accelerations
% Can find the value at any point using "ppval"
pp_IMUinG = {};
for k=1:size(path_IMUinG,2)
    pp_IMUinG{end+1} = spline(pathtime,[0; path_IMUinG(:,k); 0]);
    %pp_IMUinG{end+1} = spline(pathtime,path_IMUinG(:,k));
    %pp_IMUinG{end+1} = csapi(pathtime,path_IMUinG(:,k));
    %pp_IMUinG{end+1} = pchip(pathtime,path_IMUinG(:,k));
    %pp_IMUinG{end+1} = csape(pathtime,path_IMUinG(:,k))
end


% Our time for the states is a combo of our different sensors
% Here we will combine the two rates, so everything is in "sync"
timeimu = linspace(0,totalruntime,imurate*totalruntime);
timelidar = linspace(0,totalruntime,lidarrate*totalruntime);


%% PLANES AND ENVIROMENT DATA LOADING AND PROCESSING
% Load our data from file (convert from feet to meters)
planes2d_PinG = 0.3048.*load_2dplanedata(pathplanes);

% Convert to polygons with a height of 8ft
planes3d_PinG = planes2dtopolygons3d(planes2d_PinG, planeheight);


%% VISULIZATION AND PLOTTING OF DATA FOR ANALYSIS
% Plot the polyons (uses geom3d function)
fh = figure(2);

fontsize = 20;
set(gcf,'PaperPositionMode','auto');
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultuicontrolfontname','Bitstream Charter');
set(gcf,'DefaultAxesFontSize',fontsize);
set(gcf,'DefaultAxesFontName','Bitstream Charter');
set(gcf,'DefaultTextFontSize',fontsize);
set(gcf,'DefaultTextFontname','Bitstream Charter');

clf(fh)
for ii=1:size(planes3d_PinG,2)
	drawPolygon3d(planes3d_PinG{ii}(:,1),planes3d_PinG{ii}(:,2),planes3d_PinG{ii}(:,3),'b');
    hold on;
end

% Plot path points and their IDs
for ii=1:size(path_IMUinG,1)
    drawPoint3d(path_IMUinG(ii,1),path_IMUinG(ii,2),path_IMUinG(ii,3),'or');
    drawCoordinates3d([path_IMUinG(ii,1),path_IMUinG(ii,2),path_IMUinG(ii,3)], [path_IMUinG(ii,4),path_IMUinG(ii,5),path_IMUinG(ii,6)], 1.5)
    hold on;
end
%text(path_IMUinG(:,1),path_IMUinG(:,2),path_IMUinG(:,3),[repmat('  ',size(path_IMUinG,1),1), num2str((1:size(path_IMUinG,1))')])
hold on;

% Plot the spline (use IMU rate so it is smooth)
plot3(ppval(pp_IMUinG{1},timeimu),ppval(pp_IMUinG{2},timeimu),ppval(pp_IMUinG{3},timeimu),'r');
hold on;

% Plot lidar poses over the spline
for time=timelidar
    %drawPoint3d(ppval(pp_IMUinG{1},time),ppval(pp_IMUinG{2},time),ppval(pp_IMUinG{3},time),'og');
    hold on;
    drawCoordinates3d([ppval(pp_IMUinG{1},time),ppval(pp_IMUinG{2},time),ppval(pp_IMUinG{3},time)],...
        [ppval(pp_IMUinG{4},time),ppval(pp_IMUinG{5},time),ppval(pp_IMUinG{6},time)], 0.75)
    hold on;
end

% Draw the global axis
drawCoordinates3d([0,0,0],[0,0,0],2)


% Do the labels and the such...
axis equal
xlabel('x-axis (m)')
ylabel('y-axis (m)')
zlabel('z-axis (m)')
view([0 90])
set(gcf,'Position',[0 0 1200 400])


%% PLOT SAVE TO FILE FOR PAPER FIGURES
save_to_file = 1;
if save_to_file
    view([-25 25]);
    set(get(gca,'ylabel'),'rotation',-45)
    print(fh,'-dpng','-r500','trajectory_3d.png')
    view([0 90]);
    set(get(gca,'ylabel'),'rotation',90)
    print(fh,'-dpng','-r500','trajectory_2d_top.png')
    view([0 0]);
    print(fh,'-dpng','-r500','trajectory_2d_side.png')
end


