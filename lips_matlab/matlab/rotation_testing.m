

eul = [pi/4 pi/8 pi/2]


rotmZYX = eul2rotm(eul,'ZYX');
rotmROT = rotz(180/pi*eul(1))*roty(180/pi*eul(2))*rotx(180/pi*eul(3));



rpyanglesZYX = rotm2eul(rotmZYX,'ZYX')
rpyanglesROT = rotm2eul(rotmROT,'ZYX')


