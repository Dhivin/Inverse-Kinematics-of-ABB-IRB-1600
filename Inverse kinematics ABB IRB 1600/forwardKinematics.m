function [ A_0_1 A_1_2 A_2_3 A_3_4 A_4_5 A_5_6 ] = forwardKinematics( th1, th2, th3, th4, th5, th6 )
%This function gives out the forward kinematics transformation matrices for
%the ABB 1600-X/1.2
%The DH Parameter are as follows
%   i       Link twist     Link Length     Link Offset    Joint Angle    
%--------------------------------------------------------------------------
%   1        -90              d2             d1                 theta1*     
%   2         0               d3              0               theta2 - 90*  
%   3        -90               0              0                 theta3*     
%   4         90               0             d4                 theta4*     
%   5        -90               0              0                 theta5*     
%   6         0                0             d5                 theta6*     

global d1 d2 d3 d4 d5 sc

%Transformation matrix for frame 1 represented in frame 0
A_0_1 = make_tfMatrix(d2, -90, th1 ,d1);  
%Transformation matrix for frame 2 represented in frame 1
A_1_2 = make_tfMatrix(d3,  0, th2 - 90 ,    -162.835/sc);
%Transformation matrix for frame 3 represented in frame 2
A_2_3 = make_tfMatrix(0, -90,      th3    ,  162.835/sc);
%Transformation matrix for frame 4 represented in frame 3
A_3_4 = make_tfMatrix(0,  90,      th4    , d4); 
%Transformation matrix for frame 5 represented in frame 4
A_4_5 = make_tfMatrix(0, -90,      th5    , 0);
%Transformation matrix for frame 6 represented in frame 5
A_5_6 = make_tfMatrix(0,   0,      th6    , d5);


end

