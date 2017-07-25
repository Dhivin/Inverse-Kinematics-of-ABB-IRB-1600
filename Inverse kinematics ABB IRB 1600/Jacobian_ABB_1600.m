clc; clear ll; close all;
%% Constructing the transformation matrices for the ABB 1600
%% The angles are entered in degrees
%% make_tfMatrix function is used make_tfMatrix(a,alpha,th,d)
% where : a     = link length
%         alpha = link twist
%         th    = joint angle
%         d     = link offset

%% The link length and link offsets
global d1 d2 d3 d4 d5 sc
sc = 1e1; %scaling
d1 = 486.5/sc; d2 = 150/sc; d3 = 700/sc; d4 = 600/sc; d5 = 65/sc;


%To initialise the transformation matrices
th1 = 0; th2 = 0; th3 = 0; th4 = 0; th5 = 0; th6 = 0;
[ A_0_1, A_1_2, A_2_3, A_3_4, A_4_5, A_5_6 ] = forwardKinematics( th1, th2, th3, th4, th5, th6 );

scale = 15;
%X,Y and Z coordinates of the point from which we start writing the word or
%the sentence
x0_traj = -70.5;
y0_traj = -100;
z0_traj = 50;
%T represents the matrix which stores the orientation and position at which
%the end effector should be while writing the word.
T = [];
Jacobian = [];
J = [];
%The function phrasePath is used to generate trajectories for each alphabet
for word = 'HAT'
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
for i = 1 : tam - 1
    [th1, th2, th3, th4, th5, th6] = invKinematics2(T(:, :, i));
    [ A_0_1, A_1_2, A_2_3, A_3_4, A_4_5, A_5_6 ] = forwardKinematics( th1, th2, th3, th4, th5, th6 );
    A_0_2 = A_0_1*A_1_2;
    A_0_3 = A_0_1*A_1_2*A_2_3;
    A_0_4 = A_0_1*A_1_2*A_2_3*A_3_4;
    A_0_5 = A_0_1*A_1_2*A_2_3*A_3_4*A_4_5;
    A_0_6 = A_0_1*A_1_2*A_2_3*A_3_4*A_4_5*A_5_6;
    
    
    z = zeros(3,7);
    Jw = zeros(3,6);
    
    z0 = [0;0;1];
    z1 = A_0_1(1:3,1:3)*[0;0;1];
    z2 = A_0_2(1:3,1:3)*[0;0;1];
    z3 = A_0_3(1:3,1:3)*[0;0;1];
    z4 = A_0_4(1:3,1:3)*[0;0;1];
    z5 = A_0_5(1:3,1:3)*[0;0;1];
    z6 = A_0_6(1:3,1:3)*[0;0;1];
    
    origin0 = [0;0;0];
    origin1 = A_0_1(1:3,4);
    origin2 = A_0_2(1:3,4);
    origin3 = A_0_3(1:3,4);
    origin4 = A_0_4(1:3,4);
    origin5 = A_0_5(1:3,4);
    origin6 = A_0_6(1:3,4);
    
    J(:,1) = [cross(z0,(origin6 - origin0));z0];
    J(:,2) = [cross(z1,(origin6 - origin1));z2];
    J(:,3) = [cross(z2,(origin6 - origin2));z3];
    J(:,4) = [cross(z3,(origin6 - origin3));z4];
    J(:,5) = [cross(z4,(origin6 - origin4));z5];
    J(:,6) = [cross(z5,(origin6 - origin5));z6];
    
    Jacobian = cat(3, Jacobian, J);
end

%% Ploting joint velocities graphs

for i = 1 : 6
V(1, :) = Jacobian(1, i, :);
V(2, :) = Jacobian(2, i, :);
V(3, :) = Jacobian(3, i, :);
w(1, :) = Jacobian(4, i, :);
w(2, :) = Jacobian(5, i, :);
w(3, :) = Jacobian(6, i, :);
numberofPoint = 1 : size(Jacobian, 3);


    %Linear Velocities
    figure(i + 1);
    subplot(3, 2, 1);
    plot(numberofPoint, V(1, :), 'LineWidth', 2);
    ylabel('Vx');
    xlabel('Point');
    title(['Joint ' num2str(i) ' Vx']);
    subplot(3, 2, 3);
    plot(numberofPoint, V(2, :), 'LineWidth', 2);
    ylabel('Vy');
    xlabel('Point');
    title(['Joint ' num2str(i) ' Vy']);
    subplot(3, 2, 5);
    plot(numberofPoint, V(3, :), 'LineWidth', 2);
    ylabel('Vz');
    xlabel('Point');
    title(['Joint ' num2str(i) ' Vz']);
    %Angular Velocities
    subplot(3, 2, 2);
    plot(numberofPoint, w(1, :), 'LineWidth', 2);
    ylabel('\omegax');
    xlabel('Point');
    title(['Joint ' num2str(i) ' \omegax']);
    subplot(3, 2, 4);
    plot(numberofPoint, w(2, :), 'LineWidth', 2);
    ylabel('\omegay');
    xlabel('Point');
    title(['Joint ' num2str(i) ' \omegay']);
    subplot(3, 2, 6);
    plot(numberofPoint, w(3, :), 'LineWidth', 2);
    ylabel('\omegaz');
    xlabel('Point');
    title(['Joint ' num2str(i) ' \omegaz']);
end;
%}
