function E = bryant2angular(rpy)
% Jacobian of Omega in terms of rolldot, pitchdot and yawdot
% "Bryant Angle Representation"
% http://www.u.arizona.edu/~pen/ame553/Hallway/CAAMS/CAAMS_15.pdf

roll = rpy(1); %x-axis
pitch = rpy(2); %y-axis
yaw = rpy(3); %z-axis

% paul's version
% E = [ cos(yaw)*cos(pitch),           -sin(yaw),                   0;
%       sin(yaw)*cos(pitch),            cos(yaw),                   0;
%               -sin(pitch),                   0,                   1];

% bryant angle version
E = [   cos(roll)*cos(yaw),  sin(yaw),  0;
      -cos(pitch)*sin(yaw),  cos(yaw),  0;
                sin(pitch),         0,  1];

end
            

