function E = euler2angular(rpy)
% Jacobian of Omega in terms of rolldot, pitchdot and yawdot
% 

roll = rpy(1); %x-axis
pitch = rpy(2); %y-axis
yaw = rpy(3); %z-axis

E = [ cos(yaw)*cos(pitch),           -sin(yaw),                   0;
      sin(yaw)*cos(pitch),            cos(yaw),                   0;
              -sin(pitch),                   0,                   1];


end
            

