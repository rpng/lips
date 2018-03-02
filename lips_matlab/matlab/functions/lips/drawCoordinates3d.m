function [  ] = drawCoordinates3d( pos_FRAMEinG, rpy_FRAMEtoG, size )
%DRAWCOORDINATES3D Draws a 3D coordinate system at the given point
%   Given a position and a rpy combination we will draw a simple axis at
%   this point an time. We will make the axis as long as the specified size
%   argument.


% our rotation from the current FRAME to the GLOBAL frame
rot_FRAMEtoG = rotz(180/pi*rpy_FRAMEtoG(1,3))*roty(180/pi*rpy_FRAMEtoG(1,2))*rotx(180/pi*rpy_FRAMEtoG(1,1));

% lets find the "direction vectors" of each global axis in the new frame
delta_XinG = rot_FRAMEtoG*[size; 0; 0];
delta_YinF = rot_FRAMEtoG*[0; size; 0];
delta_ZinF = rot_FRAMEtoG*[0; 0; size];


% next up lets plot each axis
quiver3(pos_FRAMEinG(1,1),pos_FRAMEinG(1,2),pos_FRAMEinG(1,3),delta_XinG(1,1),delta_XinG(2,1),delta_XinG(3,1),0,'r','MaxHeadSize',0.5,'LineWidth',2);
hold on;
quiver3(pos_FRAMEinG(1,1),pos_FRAMEinG(1,2),pos_FRAMEinG(1,3),delta_YinF(1,1),delta_YinF(2,1),delta_YinF(3,1),0,'g','MaxHeadSize',0.5,'LineWidth',2);
hold on;
quiver3(pos_FRAMEinG(1,1),pos_FRAMEinG(1,2),pos_FRAMEinG(1,3),delta_ZinF(1,1),delta_ZinF(2,1),delta_ZinF(3,1),0,'b','MaxHeadSize',0.5,'LineWidth',2);
hold on;



end

