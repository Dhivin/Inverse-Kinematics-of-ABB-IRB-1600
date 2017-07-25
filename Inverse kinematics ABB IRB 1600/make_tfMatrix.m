function [ transformation_matrix ] = make_tfMatrix( a, alpha, th, d )
%This function takes the Denavit Hartenberg Parameters as the input and
%outputs the transformation matrix between two frames
%   a     = link length
%   aplha = link twist
%   th    = link angle
%   d     = link offset
% The angles should be input in "degrees"

transformation_matrix = [cosd(th),   -sind(th)*cosd(alpha),      sind(th)*sind(alpha),      a*cosd(th);
                         sind(th),    cosd(th)*cosd(alpha),     -cosd(th)*sind(alpha),      a*sind(th);
                              0  ,          sind(alpha)   ,          cosd(alpha)     ,           d    ;
                              0  ,             0          ,               0          ,            1   ];

end

