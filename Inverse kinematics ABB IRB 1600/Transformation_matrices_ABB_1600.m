clc; clear ll; close all;
%% Constructing the transformation matrices for the ABB 1600
%% The angles are entered in degrees
%% make_tfMatrix function is used make_tfMatrix(a,alpha,th,d)
% where : a     = link length
%         alpha = link twist
%         th    = joint angle
%         d     = link offset
%%
% The measurements here are in millimeters
d1 = 486.5; d2 = 150; d3 = 700; d4 = 600; d5 = 65;

%syms th1 th2 th3 th4 th5 th6;
th1 = 0; th2 =0; th3=0; th4=0; th5=0; th6=0;
%Transformation matrix for frame 1 represented in frame 0
A_0_1 = make_tfMatrix(d2, -90,      th1    ,d1) 
%Transformation matrix for frame 2 represented in frame 1
A_1_2 = make_tfMatrix(d3,  0, th2 - 90 ,    -162.835/sc)
%Transformation matrix for frame 3 represented in frame 2
A_2_3 = make_tfMatrix(0, -90,      th3    ,  162.835/sc)
%Transformation matrix for frame 4 represented in frame 300
A_3_4 = make_tfMatrix(0,  90,      th4    , d4) 
%Transformation matrix for frame 5 represented in frame 4
A_4_5 = make_tfMatrix(0, -90,      th5    , 0)
%Transformation matrix for frame 6 represented in frame 5
A_5_6 = make_tfMatrix(0,   0,      th6    , d5)

%Tranformation matrix for frame 6 represented in frame 0
A_0_6 = A_0_1*A_1_2*A_2_3*A_3_4*A_4_5*A_5_6