% This script brings the imported links to the base frame position and
% orientation. This is necessary because the links imported from Solidworks
% in the STL format have their own co-ordinate system and hence the frames
% which are attached to the links during forward kinematics are not at the correct 
% positions in regards to the base frame of the robot. 
%
%Hence this script brings the frames attached to the robot during
%forward kinematics in the same orientation and position as the base frame 
%so that they can be moved to their correct positions and orientations to
%form the robot.


% Moving link1(the base frame) to align with frame 0
T = [1    0    0   (-382.1/sc)
    0    1    0    (-240.3/sc)
    0    0    1    0
    0    0    0    1];
link1_moved = moveToOrigin(link1, T);

% Moving link2 to align with frame 0
T = [1    0    0   (-413.4/sc)
    0    1    0    (-177.75/sc)
    0    0    1    -484.6/sc
    0    0    0    1];
link2_moved = moveToOrigin(link2, T);

T = [1    0    0   0
    0    0    -1    0
    0    1    0    0
    0    0    0    1];
link2_moved = moveToOrigin(link2_moved, T);

% Moving link3 to align with frame 0
T = [1    0    0   (-150/sc)
    0    1    0    -97.835/sc
    0    0    1    -1186.5/sc
    0    0    0    1];
link3_moved = moveToOrigin(link3, T);

T = [0    0    1   0
    1    0     0   0
    0    1    0    0
    0    0    0    1];
link3_moved = moveToOrigin(link3_moved, T);

% Moving link4 to align with frame 0     
T = [1    0    0   (-176.94/sc)
    0    1    0    -119.3/sc
    0    0    1    -1186.5  /sc
    0    0    0    1];
link4_moved = moveToOrigin(link4, T);

T = [0   0    1   0
     0   -1    0   0
     1   0    0   0
     0   0    0   1];
link4_moved = moveToOrigin(link4_moved, T);

%Moving link5 to align with frame 0    
T = [1    0    0   (-286.7/sc)
    0    1    0    -62.1/sc
    0    0    1    -57.1/sc
    0    0    0    1];
link5_moved = moveToOrigin(link5, T);

T = [0   0    1   0
     1   0    0   0
     0   1    0   0
     0   0    0   1];
link5_moved = moveToOrigin(link5_moved, T);

%Moving link6 to align with frame 0    
T = [1    0    0   (-33.008/sc)
    0    1    0    -23.50/sc
    0    0    1    -33.008/sc
    0    0    0    1];
link6_moved = moveToOrigin(link6, T);

T = [0   0    1   0
     0  -1    0   0
     1   0    0   0
     0   0    0   1];
link6_moved = moveToOrigin(link6_moved, T);

%Moving link7 to align with frame 0    
T = [1    0    0   (-815/sc)
    0    1    0     -25/sc
    0    0    1     -25/sc
    0    0    0    1];
link7_moved = moveToOrigin(link7, T);

T = [0   0    1   0
     0  -1    0   0
     1   0    0   0
     0   0    0   1];
link7_moved = moveToOrigin(link7_moved, T);



