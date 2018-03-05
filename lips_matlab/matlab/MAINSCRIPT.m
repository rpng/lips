% Clear and close everything
clear all;
close all;

% Add paths needed
addpath('functions/matGeom/polygons2d/')
addpath('functions/matGeom/geom3d/')
addpath('functions/lips/')

%% CONFIG INFORMATION IS ALL IN HERE

showrealtimeplot = 0; %if we should display a realtime plot, 0=faster

pathplanes = '../input/floorplan_spencer_small.txt';
pathpath = '../input/path_spencer_small_01.txt';

planeheight = 3; %meters

imurate = 500; %hz
lidarrate = 1; %hz

gravityglobal = [0,0,9.81]; %m/s^2

totalruntime = 200; %seconds

lidarzenith = [3.197,0.000,-3.197,-6.360,-9.465,-12.491,-15.424,-18.249]; %deg
%lidarzenith = [0.000]; %deg
lidarresolution = 0.25; %deg

timestill = 3; %seconds we should be stationary in the begining

R_LtoI = [-1,0,0;0,1,0;0,0,-1]; % 180 deg rot around y-axis
p_IinL = [0;0.04;-0.06]; % 4cm in y-direction, and -6cm in the z-direction


%% ROBOT TRAJECTORY LOADING AND PROCESSING
% Debug
fprintf('MAIN: Loading our control points\n')

% Load our data from file
data = csvread(pathpath,1,0);
%pathtime = linspace(0,totalruntime,size(path_IMUinG,1));
pathtime = totalruntime*data(:,1)/max(data(:,1));
path_IMUinG = data(:,2:end);

% NOTE: Convert from feet poses to meters
path_IMUinG(:,1:3) = 0.3048.*path_IMUinG(:,1:3);

% NOTE: Convert from degrees to radians
path_IMUinG(:,4:6) = pi/180.*path_IMUinG(:,4:6);

% Create a spline for each dimension x,y,z and roll,pitch,yaw
% This will be used to evaluate the derivative to give us accelerations
% Can find the value at any point using "ppval"
% NOTE: WE FORCE WE START WITH ZERO VELOCITY BY "CLAMPING" THE SPLINE
pp_IMUinG = {};
for k=1:size(path_IMUinG,2)
    pp_IMUinG{end+1} = spline(pathtime,[0; path_IMUinG(:,k); 0]);
    %pp_IMUinG{end+1} = csapi(pathtime,path_IMUinG(:,k));
    %pp_IMUinG{end+1} = pchip(pathtime,path_IMUinG(:,k));
end

% Debug
fprintf('MAIN: Generating spline trajectory\n')

% Calculate the step size for each sensor
stepimu = 1/imurate;
steplidar = 1/lidarrate;

% Our time for the states is a combo of our different sensors
% Here we will combine the two rates, so everything is in "sync"
timeimu = 0:stepimu:totalruntime;
timelidar = 0:steplidar:totalruntime;

% Next up, get the IMU pose and its derivaties (i.e. the ACC readings)
trajectory.dataimu.time = timeimu;
trajectory.dataimu.x = ppval(pp_IMUinG{1},trajectory.dataimu.time);
trajectory.dataimu.y = ppval(pp_IMUinG{2},trajectory.dataimu.time);
trajectory.dataimu.z = ppval(pp_IMUinG{3},trajectory.dataimu.time);
trajectory.dataimu.velx = ppval(fnder(pp_IMUinG{1},1),trajectory.dataimu.time);
trajectory.dataimu.vely = ppval(fnder(pp_IMUinG{2},1),trajectory.dataimu.time);
trajectory.dataimu.velz = ppval(fnder(pp_IMUinG{3},1),trajectory.dataimu.time);
trajectory.dataimu.accelx = ppval(fnder(pp_IMUinG{1},2),trajectory.dataimu.time);
trajectory.dataimu.accely = ppval(fnder(pp_IMUinG{2},2),trajectory.dataimu.time);
trajectory.dataimu.accelz = ppval(fnder(pp_IMUinG{3},2),trajectory.dataimu.time);

% Next up, get the IMU orientation and its derivaties (i.e. the GYRO readings)
% Note: convert from deg to radians
% Note: will later convert euler angular to the correct body centric angular
% Note: will later convert from the IMU frame to the LIDAR frame
trajectory.dataimu.roll = ppval(pp_IMUinG{4},trajectory.dataimu.time);
trajectory.dataimu.pitch = ppval(pp_IMUinG{5},trajectory.dataimu.time);
trajectory.dataimu.yaw = ppval(pp_IMUinG{6},trajectory.dataimu.time);
trajectory.dataimu.angularroll = ppval(fnder(pp_IMUinG{4},1),trajectory.dataimu.time); %convert from rpy to measurement later
trajectory.dataimu.angularpitch = ppval(fnder(pp_IMUinG{5},1),trajectory.dataimu.time); %convert from rpy to measurement later
trajectory.dataimu.angularyaw = ppval(fnder(pp_IMUinG{6},1),trajectory.dataimu.time); %convert from rpy to measurement later

% Store our inital IMU parameters
posx = trajectory.dataimu.x(1);
posy = trajectory.dataimu.y(1);
posz = trajectory.dataimu.z(1);
orir = trajectory.dataimu.roll(1);
orip = trajectory.dataimu.pitch(1);
oriy = trajectory.dataimu.yaw(1);


% Append our stationary time for the IMU data
trajectory.dataimu.time = trajectory.dataimu.time + timestill;
fprintf('MAIN: %d stationary IMU poses.\n',length(timestill-stepimu:-stepimu:0))
for time=timestill-stepimu:-stepimu:0
    trajectory.dataimu.time = [time trajectory.dataimu.time];
    trajectory.dataimu.x = [posx trajectory.dataimu.x];
    trajectory.dataimu.y = [posy trajectory.dataimu.y];
    trajectory.dataimu.z = [posz trajectory.dataimu.z];
    trajectory.dataimu.velx = [0 trajectory.dataimu.velx];
    trajectory.dataimu.vely = [0 trajectory.dataimu.vely];
    trajectory.dataimu.velz = [0 trajectory.dataimu.velz];
    trajectory.dataimu.accelx = [0 trajectory.dataimu.accelx];
    trajectory.dataimu.accely = [0 trajectory.dataimu.accely];
    trajectory.dataimu.accelz = [0 trajectory.dataimu.accelz];
    trajectory.dataimu.roll = [orir trajectory.dataimu.roll];
    trajectory.dataimu.pitch = [orip trajectory.dataimu.pitch];
    trajectory.dataimu.yaw = [oriy trajectory.dataimu.yaw];
    trajectory.dataimu.angularroll = [0 trajectory.dataimu.angularroll];
    trajectory.dataimu.angularpitch = [0 trajectory.dataimu.angularpitch];
    trajectory.dataimu.angularyaw = [0 trajectory.dataimu.angularyaw];
end


% We should append the gravity vector rotated into the IMU frame
% Also we need to convert our angular roll,pitch,yaw to the rad/s about the instantaneous axis of rotation
fprintf('MAIN: Creating IMU measurements from RPY and global ACCEL.\n')
for ii=1:size(trajectory.dataimu.time,2)
    % Find acceleration + gravity in global frame then convert to LOCAL
    rot_ItoG = rotz(180/pi*trajectory.dataimu.yaw(1,ii))*roty(180/pi*trajectory.dataimu.pitch(1,ii))*rotx(180/pi*trajectory.dataimu.roll(1,ii));
    accelinG = [trajectory.dataimu.accelx(1,ii);trajectory.dataimu.accely(1,ii);trajectory.dataimu.accelz(1,ii)] + gravityglobal';
    accelinL = rot_ItoG'*accelinG;
    % Update our acceleration
    trajectory.dataimu.accelx(1,ii) = accelinL(1);
    trajectory.dataimu.accely(1,ii) = accelinL(2);
    trajectory.dataimu.accelz(1,ii) = accelinL(3);
    % Convet the RPY angular to the actual LOCAL imu measurement
    % See: "Time Derivative of Bryant Angles"
    % http://www.u.arizona.edu/~pen/ame553/Hallway/CAAMS/CAAMS_15.pdf
    rpy = [trajectory.dataimu.roll(1,ii);trajectory.dataimu.pitch(1,ii);trajectory.dataimu.yaw(1,ii)];
    %angvel = bryant2angular(rpy)*[trajectory.dataimu.angularroll(1,ii); trajectory.dataimu.angularpitch(1,ii);trajectory.dataimu.angularyaw(1,ii)];
    angvel = rot_ItoG'*euler2angular(rpy)*[trajectory.dataimu.angularroll(1,ii); trajectory.dataimu.angularpitch(1,ii);trajectory.dataimu.angularyaw(1,ii)];
    % Save this to file
    trajectory.dataimu.angularroll(1,ii) = angvel(1);
    trajectory.dataimu.angularpitch(1,ii) = angvel(2);
    trajectory.dataimu.angularyaw(1,ii) = angvel(3);
end


% Next up, get the LIDAR pose information, this is not used by the filter
% Note: convert from degrees to radians
% NOTE: convert from the floorplane feet to meters
trajectory.datalidar.time = timelidar;
trajectory.datalidar.x = ppval(pp_IMUinG{1},trajectory.datalidar.time);
trajectory.datalidar.y = ppval(pp_IMUinG{2},trajectory.datalidar.time);
trajectory.datalidar.z = ppval(pp_IMUinG{3},trajectory.datalidar.time);
trajectory.datalidar.roll = ppval(pp_IMUinG{4},trajectory.datalidar.time);
trajectory.datalidar.pitch = ppval(pp_IMUinG{5},trajectory.datalidar.time);
trajectory.datalidar.yaw = ppval(pp_IMUinG{6},trajectory.datalidar.time);

% Store our inital LIDAR parameters
posx = trajectory.datalidar.x(1);
posy = trajectory.datalidar.y(1);
posz = trajectory.datalidar.z(1);
orir = trajectory.datalidar.roll(1);
orip = trajectory.datalidar.pitch(1);
oriy = trajectory.datalidar.yaw(1);

% Append our stationary time for the LIDAR data
trajectory.datalidar.time = trajectory.datalidar.time + timestill;
fprintf('MAIN: %d stationary lidar poses.\n',length(timestill-steplidar:-steplidar:0))
for time=timestill-steplidar:-steplidar:0
    trajectory.datalidar.time = [time trajectory.datalidar.time];
    trajectory.datalidar.x = [posx trajectory.datalidar.x];
    trajectory.datalidar.y = [posy trajectory.datalidar.y];
    trajectory.datalidar.z = [posz trajectory.datalidar.z];
    trajectory.datalidar.roll = [orir trajectory.datalidar.roll];
    trajectory.datalidar.pitch = [orip trajectory.datalidar.pitch];
    trajectory.datalidar.yaw = [oriy trajectory.datalidar.yaw];    
end


% Loop through each LIDAR measurement, and lets transform it from the IMU
% frame into the correct LIDAR frame of reference
fprintf('MAIN: Converting IMU poses to LIDAR poses.\n')
for ii=1:size(trajectory.datalidar.time,2)
    % Convert position of the IMU into the LIDAR frame
    % EQ: p_LinG = p_IinG - R_ItoG*R_LtoI*p_IinL
    p_IinG = [trajectory.datalidar.x(ii); trajectory.datalidar.y(ii); trajectory.datalidar.z(ii)];
    R_ItoG = rotz(180/pi*trajectory.datalidar.yaw(ii))*roty(180/pi*trajectory.datalidar.pitch(ii))*rotx(180/pi*trajectory.datalidar.roll(ii));
    p_LinG = p_IinG - R_ItoG*R_LtoI*p_IinL;
    % Convert orientation from IMU into the LIDAR
    % EQ: R_LtoG = R_ItoG*R_LtoI
    R_LtoG = R_ItoG*R_LtoI;
    % Save these, and replace the current LIDAR pose values
    trajectory.datalidar.x(ii) = p_LinG(1);
    trajectory.datalidar.y(ii) = p_LinG(2);
    trajectory.datalidar.z(ii) = p_LinG(3);
    rpyangles = rotm2eul(R_LtoG,'ZYX');
    trajectory.datalidar.yaw(ii) = rpyangles(1);
    trajectory.datalidar.pitch(ii) = rpyangles(2);
    trajectory.datalidar.roll(ii) = rpyangles(3);
end


% Next, create the location that we will store our lidar information
trajectory.datalidar.planeids = {};
trajectory.datalidar.rayids = {};
trajectory.datalidar.points = {};



%% PLANES AND ENVIROMENT DATA LOADING AND PROCESSING
% Load our data from file (convert from feet to meters)
planes2d_PinG = 0.3048.*load_2dplanedata(pathplanes);

% Convert to polygons to use the given height
planes3d_PinG = planes2dtopolygons3d(planes2d_PinG, planeheight);

% Add the ground plane at z=0 as a polygon
planes3d_PinG = addhorzplane(planes2d_PinG, planes3d_PinG, 0);

% Add the ceiling plane at z=0 as a polygon
planes3d_PinG = addhorzplane(planes2d_PinG, planes3d_PinG, planeheight);


%% NOW FOR EACH LIDAR STATE WE WILL GET THE MEASUREMENTS

if showrealtimeplot
fg1 = figure('Name','LIDAR ray-intersections');
end

% loop through each lidar state
for zz=1:size(trajectory.datalidar.time,2)
    % total variables we will sum
    hits = [];
    polyids = [];
    rayids = [];
    % 1. get position from our spline
    pos_LIDARinG = [trajectory.datalidar.x(zz),trajectory.datalidar.y(zz),trajectory.datalidar.z(zz)];
    % 2. get current rotation
    rpy_LIDARtoG = [trajectory.datalidar.roll(zz),trajectory.datalidar.pitch(zz),trajectory.datalidar.yaw(zz)];
    % start timer
    tic;
    % loop through each lidar scan line (360 degrees)
    %delete(gcp('nocreate'));
    % Set the properties of the cluster
    %myCluster = parcluster('local');
    %myCluster.NumWorkers = size(lidarzenith,2);
    %saveProfile(myCluster);
    %parpool('local',size(lidarzenith,2));
    %parfor ii=1:size(lidarzenith,2)
    for ii=1:size(lidarzenith,2)
        % get our angle
        zenith = lidarzenith(1,ii);
        % 3. generate our rays
        rays_inG = gencirclerays(pos_LIDARinG, rpy_LIDARtoG, zenith, lidarresolution, 1);
        % 4. intersect the rays
        %[hitst, polyidst, rayidst] = intersectrayspolys(rays_inG, planes3d_PinG);
        [hitst, polyidst, rayidst] = kevinsPlaneIntersection(rays_inG, planes3d_PinG);
        % 5. remove invalid points
        [hitst, polyidst, rayidst] = invalidateintersections(pos_LIDARinG, hitst, polyidst, rayidst);
        % 6. append this scan ring to our measurement
        hits = [hits; hitst];
        polyids = [polyids; polyidst];
        rayids = [rayids; rayidst];
        % Make sure our figure updates
        drawnow;
    end
    % 7. append to our master trajectory object
    [polyids, indices] = sort(polyids);
    hits = hits(indices,:);
    rayids = rayids(indices,:);
    trajectory.datalidar.planeids{end+1} = polyids;
    trajectory.datalidar.rayids{end+1} = rayids;
    trajectory.datalidar.points{end+1} = hits;
    % 8. plot for the user to see
    if showrealtimeplot
        set(0, 'CurrentFigure', fg1)
        clf(fg1)
        % Plot the polygons
        for ii=1:size(planes3d_PinG,2)
            drawPolygon3d(planes3d_PinG{ii}(:,1),planes3d_PinG{ii}(:,2),planes3d_PinG{ii}(:,3),'b');
            hold on;
        end
        % Plot the intersection points and a ray to them
        for ii=1:size(hits,1)
            drawPoint3d(hits(ii,1),hits(ii,2),hits(ii,3),'.-k');
            hold on;
            %drawPolyline3d([pos_IMUinG(1,1),hits(ii,1)],[pos_IMUinG(1,2),hits(ii,2)],[pos_IMUinG(1,3),hits(ii,3)],'-k')
            hold on;
        end
        % Draw current position
        drawPoint3d(pos_LIDARinG(1,1),pos_LIDARinG(1,2),pos_LIDARinG(1,3),'or'); hold on;
        drawCoordinates3d([pos_LIDARinG(1,1),pos_LIDARinG(1,2),pos_LIDARinG(1,3)],[rpy_LIDARtoG(1,1),rpy_LIDARtoG(1,2),rpy_LIDARtoG(1,3)],1); hold on;
        % Draw the global frame
        drawCoordinates3d([0,0,0],[0,0,0],2)   
        % Plot path points and their IDs (the given waypoints in IMU frame)
        for ii=1:size(path_IMUinG,1)
            drawPoint3d(path_IMUinG(ii,1),path_IMUinG(ii,2),path_IMUinG(ii,3),'or'); hold on;
            drawCoordinates3d([path_IMUinG(ii,1),path_IMUinG(ii,2),path_IMUinG(ii,3)],[path_IMUinG(ii,4),path_IMUinG(ii,5),path_IMUinG(ii,6)],1); hold on;
        end
        text(path_IMUinG(:,1),path_IMUinG(:,2),path_IMUinG(:,3),[repmat('  ',size(path_IMUinG,1),1), num2str((1:size(path_IMUinG,1))')])
        hold on;
        % Plot the spline (use IMU rate so it is smooth)
        plot3(ppval(pp_IMUinG{1},timeimu),ppval(pp_IMUinG{2},timeimu),ppval(pp_IMUinG{3},timeimu),'r');
        hold on;
        % Do the labels and the such...
        axis equal
        xlabel('x-direction (meters)')
        ylabel('y-direction (meters)')
        zlabel('z-direction (meters)')
        %view([0 90])
        drawnow;
    end
    % end timer and print
    fprintf('MAIN: %f seconds to process\n',toc)
end



%% EXPORT DATA TO FILE FOR USE IN MSCKF CODE

% debug info
disp('DATA: Exporting data to disk')

% ensure our output directory exists
if ~exist('../output/', 'dir')
    disp('DATA: Creating "output" directory...')
    mkdir('../output/');
end


% our static transform
dlmwrite('../output/config_transform.txt', R_LtoI(:)', 'precision', 16);
dlmwrite('../output/config_transform.txt',p_IinL','-append', 'precision', 16);

% our runtime
dlmwrite('../output/config_runtime.txt', totalruntime+timestill);

% our lidar data
dlmwrite('../output/config_lidar.txt', lidarzenith, 'precision', 16);
dlmwrite('../output/config_lidar.txt',lidarresolution,'-append', 'precision', 16);

% our gravity
dlmwrite('../output/config_gravity.txt', gravityglobal, 'precision', 16);

% our rates
dlmwrite('../output/config_rates.txt', [imurate lidarrate], 'precision', 16);
dlmwrite('../output/config_rates.txt', [stepimu steplidar],'-append', 'precision', 16);

% imu true pose
dlmwrite('../output/true_poseimu.txt', [trajectory.dataimu.time' trajectory.dataimu.x' trajectory.dataimu.y' trajectory.dataimu.z'...
                                    trajectory.dataimu.roll' trajectory.dataimu.pitch' trajectory.dataimu.yaw'], 'precision', 16);
% imu true readings
dlmwrite('../output/true_measimu.txt', [trajectory.dataimu.time' trajectory.dataimu.accelx' trajectory.dataimu.accely' trajectory.dataimu.accelz'...
                            trajectory.dataimu.angularroll' trajectory.dataimu.angularpitch' trajectory.dataimu.angularyaw'], 'precision', 16);
% lidar true pose
dlmwrite('../output/true_poselidar.txt', [trajectory.datalidar.time' trajectory.datalidar.x' trajectory.datalidar.y' trajectory.datalidar.z'...
                                     trajectory.datalidar.roll' trajectory.datalidar.pitch' trajectory.datalidar.yaw'], 'precision', 16);                                
% lidar true IDs (plane IDs)
fid = fopen('../output/true_idsplanes.txt','wt');
for ii=1:size(trajectory.datalidar.time,2)
      fprintf(fid,'%.15f,',trajectory.datalidar.time(1,ii));
      fprintf(fid,'%d,',trajectory.datalidar.planeids{ii}(1:end-1));
      fprintf(fid,'%d\n',trajectory.datalidar.planeids{ii}(end));
end
fclose(fid);
% lidar true measurements (3d points)
fid = fopen('../output/true_measplanes.txt','wt');
for ii=1:size(trajectory.datalidar.time,2)
    % timestamp
    time = trajectory.datalidar.time(1,ii);
    fprintf(fid,'%.15f,', time);
    % current pose
    p_LinG = [trajectory.datalidar.x(ii),trajectory.datalidar.y(ii),trajectory.datalidar.z(ii)]';
    rot_LtoG = rotz(180/pi*trajectory.datalidar.yaw(ii))*roty(180/pi*trajectory.datalidar.pitch(ii))*rotx(180/pi*trajectory.datalidar.roll(ii));
    % loop through all points, and transform into the local lidar frame
    for jj=1:size(trajectory.datalidar.points{ii},1)-1
        ptinL = rot_LtoG'*(trajectory.datalidar.points{ii}(jj,:)' - p_LinG);
        fprintf(fid,'%.15f,%.15f,%.15f,',ptinL(1,1),ptinL(2,1),ptinL(3,1));
    end
    % finally do the last point
    ptinL = rot_LtoG'*(trajectory.datalidar.points{ii}(end,:)' - p_LinG);
    fprintf(fid,'%.15f,%.15f,%.15f\n',ptinL(1,1),ptinL(2,1),ptinL(3,1));
end
fclose(fid);

% figure of the trajectory
if showrealtimeplot
    set(0, 'CurrentFigure', fg1)
    set(gcf,'Position',[0 0 900 600])
    print(fg1,'-dpng','-r500','../output/trajectory.png')
end

% debug info
disp('DATA: Done saving to disk!')





