function [ theta1 theta2 theta3 theta4 theta5 theta6 ] = invKinematics2( Transformation_given )
%This function is used for inverse kinematics calculations.

%link lenghts in mm
global d1 d2 d3 d4 d5

alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];
a = [d2 d3 0 0 0 0];
d = [d1 0 0 d4 0 d5];

%%%%%%%%%%%%%%%%%Inverse kinematics%%%%%%%%%%%%%%%%%
x = Transformation_given(1,4);
y = Transformation_given(2,4);
z = Transformation_given(3,4);
R= Transformation_given(1:3,1:3);

xc = x - d5*R(1,3);
yc = y - d5*R(2,3);
zc = z - d5*R(3,3);
r = sqrt(xc^2 + yc^2);

%Inverse kinematic equations for position
theta1 = atan2(yc,xc);

r = (xc^2 + yc^2)^(1/2);
D = ((r - d2)^2 + (zc - d1)^2 - d3^2 - d4^2)/(2*d3*d4);

theta3_dash = atan2((-(1 - D^2)^(1/2)), D);
theta3 = abs(theta3_dash) - (90*pi/180);

theta2_dash = (atan2( zc-d1, r-d2) - atan2(d4*sind(theta3_dash*180/pi),d3+d4*cosd(theta3_dash*180/pi))) ;
theta2 = (90*pi/180) - theta2_dash;

theta5 = atan2(sqrt(1-(sin(theta1)*R(1,3)- cos(theta1)*R(2,3))^2),sin(theta1)*R(1,3) - cos(theta1)*R(2,3)) -pi;
theta4 = atan2(-cos(theta1)*sin(theta2+theta3)*R(1,3)-sin(theta1)*sin(theta2+theta3)*R(2,3)+cos(theta2+theta3)*R(3,3),cos(theta1)*cos(theta2+theta3)*R(1,3)+sin(theta1)*cos(theta2+theta3)*R(2,3)+sin(theta2+theta3)*R(3,3));
theta6= atan2(sin(theta1)*R(1,2)-cos(theta1)*R(2,2),-sin(theta1)*R(1,1)+cos(theta1)*R(2,1));

%Converting angles to degrees, because angles passed as arguments to the
%function forwardKinematics() have to be in degrees.
theta1 = theta1 * 180 / pi;
theta2 = theta2 * 180 / pi;
theta3 = theta3 * 180 / pi;
theta4 = theta4 * 180 / pi;
theta5 = theta5 * 180 / pi;
theta6 = theta6 * 180 / pi;

end

