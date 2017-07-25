clc; clear all; close all;
%% Robot Dynamics Course Project
% In this project we simulate the ABB 1600 X/1.2 robot manipulator to write
% words. First we import Solidwork models as .STL files into MATLAB, move
% them to the correct positions with respect to the base. And then we
% generate trajectories required to write the words or sentences and
% perform inverse kinematics to find joint angles, and use Jacobians to
% find joint velocities. Then we move the robot links to the required
% position.

%% The link length and link offsets
global d1 d2 d3 d4 d5 sc
sc = 1e1; %scaling
d1 = 486.5/sc; d2 = 150/sc; d3 = 700/sc; d4 = 600/sc; d5 = 65/sc;

%% Reading the STL files

link1 = stlread('link1.stl'); link1.facecolor = [0.1 0.1 0.1]; link1.edgecolor = 'none'; link1.vertices = link1.vertices / sc;
link2 = stlread('link2.stl'); link2.facecolor = [0.2 0.2 0.2]; link2.edgecolor = 'none'; link2.vertices = link2.vertices / sc;
link3 = stlread('link3.stl'); link3.facecolor = [0.3 0.3 0.3]; link3.edgecolor = 'none'; link3.vertices = link3.vertices / sc;
link4 = stlread('link4.stl'); link4.facecolor = [0.4 0.4 0.4]; link4.edgecolor = 'none'; link4.vertices = link4.vertices / sc;
link5 = stlread('link5.stl'); link5.facecolor = [0.5 0.5 0.5]; link5.edgecolor = 'none'; link5.vertices = link5.vertices / sc;
link6 = stlread('link6.stl'); link6.facecolor = [0.5 0.5 0.5]; link6.edgecolor = 'none'; link6.vertices = link6.vertices / sc;
link7 = stlread('link7.stl'); link7.facecolor = [0.5 0.5 0.5]; link7.edgecolor = 'none'; link7.vertices = link7.vertices / sc;

%% Initial transformation equations.
%The transformation matrices calculated here are used for moving the links
%to their correct positions with respect to the base in the simulation. The
%DH parameters used here are different from the ones used in the inverse
%kinematics equations. There are basically two differences. The link offset
%-162.635 and 162.835 for the link 2 and 3  respectively are taken as 0
%in the inverse kinematics, to make the calculations easy.

th1 = 0; th2 = 0; th3 = 0; th4 = 0; th5 = 0; th6 = 0;
[ A_0_1, A_1_2, A_2_3, A_3_4, A_4_5, A_5_6 ] = forwardKinematics( th1, th2, th3, th4, th5, th6 );

A_0_2 = A_0_1*A_1_2;
A_0_3 = A_0_1*A_1_2*A_2_3;
A_0_4 = A_0_1*A_1_2*A_2_3*A_3_4;
A_0_5 = A_0_1*A_1_2*A_2_3*A_3_4*A_4_5;
A_0_6 = A_0_1*A_1_2*A_2_3*A_3_4*A_4_5*A_5_6;

%% Moving the links to their correct position in the MATLAB simulation

bringLinksToOrigin();

link2_current = homogeneousTransformation(link2_moved, A_0_1);
link3_current = homogeneousTransformation(link3_moved, A_0_2);
link4_current = homogeneousTransformation(link4_moved, A_0_3);
link5_current = homogeneousTransformation(link5_moved, A_0_4);
link6_current = homogeneousTransformation(link6_moved, A_0_5);
link7_current = homogeneousTransformation(link7_moved, A_0_6);

%% Creating Trajectory and moving robot

scale = 15;
%X,Y and Z coordinates of the point from which we start writing the word or
%the sentence
x0_traj = -70.5;
y0_traj = -100;
z0_traj = 50;
%T represents the matrix which stores the orientation and position at which
%the end effector should be while writing the word.
T = [];

%The function phrasePath is used to generate trajectories for each alphabet
for word = 'H H H'
    if word ~= ' '
        T = cat(3, T, phrasePath(word, scale, x0_traj, y0_traj, z0_traj));
    end;
    x0_traj = x0_traj + .9 * scale;
    if word == '`'
        x0_traj = x0_traj - .2 * scale;
    elseif word == '!'
        x0_traj = x0_traj - .4 * scale;
    end;
end;

tam = size(T, 3);
figure(1);
for i = 1 : tam - 1
    [th1, th2, th3, th4, th5, th6] = invKinematics2(T(:, :, i));
    
    theta(:,i) = [th1, th2, th3, th4, th5, th6];
    [ A_0_1, A_1_2, A_2_3, A_3_4, A_4_5, A_5_6 ] = forwardKinematics( th1, th2, th3, th4, th5, th6 );
    
    A_0_2 = A_0_1*A_1_2;
    A_0_3 = A_0_1*A_1_2*A_2_3;
    A_0_4 = A_0_1*A_1_2*A_2_3*A_3_4;
    A_0_5 = A_0_1*A_1_2*A_2_3*A_3_4*A_4_5;
    A_0_6 = A_0_1*A_1_2*A_2_3*A_3_4*A_4_5*A_5_6;
    
    link2_current = homogeneousTransformation(link2_moved, A_0_1);
    link3_current = homogeneousTransformation(link3_moved, A_0_2);
    link4_current = homogeneousTransformation(link4_moved, A_0_3);
    link5_current = homogeneousTransformation(link5_moved, A_0_4);
    link6_current = homogeneousTransformation(link6_moved, A_0_5);
    link7_current = homogeneousTransformation(link7_moved, A_0_6);
    
    %Draw the letters
    cla, hold on;
    patch(link1_moved);
    patch(link2_current);
    patch(link3_current);
    patch(link4_current);
    patch(link5_current);
    patch(link6_current);
    patch(link7_current);
    if T(2, 4, i) == y0_traj
        referenceTrajectory(i, 1) = T(1, 4, i); referenceTrajectory(i, 2) = T(2, 4, i); referenceTrajectory(i, 3) = T(3, 4, i);
        plot3(referenceTrajectory(:, 1), referenceTrajectory(:, 2), referenceTrajectory(:, 3), '.b');
        actualTrajectory(i, 1) = A_0_6(1, 4); actualTrajectory(i, 2) = A_0_6(2, 4); actualTrajectory(i, 3) = A_0_6(3, 4);
        plot3(actualTrajectory(:, 1), actualTrajectory(:, 2), actualTrajectory(:, 3), '*r');
    end;
    hold off, axis equal;
    maximum = d1 + d2 + d3 + d4 + d5;
    minimum = -maximum;
    axis([minimum maximum minimum maximum 0 maximum]);
    view(3), xlabel('x axis'), ylabel('y axis'), zlabel('z axis'), grid on;
    pause(.0001);
    
end

%}
