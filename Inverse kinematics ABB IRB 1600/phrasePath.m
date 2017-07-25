function T = phrasePath(alphabet, scale, x0_traj, y0_traj, z0_traj)

aux = 3;
scale = scale * aux;
x0_traj = x0_traj * aux;
y0_traj = y0_traj * aux;
z0_traj = z0_traj * aux;

% Parameters to build each letter
cte = .01 * scale;
alphabetHorizontalSize = floor(.6 * scale);
alphabetVerticalSize = floor(scale);

x = x0_traj;
y = y0_traj + cte;
z = z0_traj;

switch alphabet
%% This section is used to compute the x,y and z coordinates for the trajectory for alphabets
% here we have written the formulas for calculating x,y and z coordinates
% for three alphabets T,H and A. Similarly we can generate trajectories for
% all the english alphabets and also any other language alphabets. 
    case 'T'
        % first part - the vertical line drawing it down to up
        x = cat(2, 2, x, (x0_traj + alphabetHorizontalSize / 2) * ones(1, alphabetVerticalSize + 1));
        y = cat(2, 2, y, y0_traj * ones(1, alphabetVerticalSize + 1));
        z = cat(2, 2, z, z0_traj : z0_traj + alphabetVerticalSize);

        % before go to next part we move to the end of the horizontal line
        x = cat(2, x, x(end));
        y = cat(2, y, y0_traj + cte);
        z = cat(2, z, z(end));

        % this part is what computes x,y and z for the 
        x = cat(2, x, x0_traj : x0_traj + alphabetHorizontalSize);
        y = cat(2, y, y0_traj * ones(1, alphabetHorizontalSize + 1));
        z = cat(2, z, (z0_traj + alphabetVerticalSize) * ones(1, alphabetHorizontalSize + 1));
    case 'H'
        % first part - vertical line
        x = cat(2, x, x0_traj * ones(1, alphabetVerticalSize + 1));
        y = cat(2, y, y0_traj * ones(1, alphabetVerticalSize + 1));
        z = cat(2, z, z0_traj : z0_traj + alphabetVerticalSize);

        % before go to next part move to the center of the vertical line
        x = cat(2, x, x(end));
        y = cat(2, y, y0_traj + cte);
        z = cat(2, z, z(end));

        % draw the horizontal line in the middel
        x = cat(2, x, x0_traj : x0_traj + alphabetHorizontalSize);
        y = cat(2, y, y0_traj * ones(1, alphabetHorizontalSize + 1));
        z = cat(2, z, (z0_traj + alphabetVerticalSize / 2) * ones(1, alphabetHorizontalSize + 1));

        % move to the bottom of the vertical line
        x = cat(2, x, x(end));
        y = cat(2, y, y0_traj + cte);
        z = cat(2, z, z(end));

        % draws the vertical line
        x = cat(2, x, (x0_traj + alphabetHorizontalSize) * ones(1, alphabetVerticalSize + 1));
        y = cat(2, y, y0_traj * ones(1, alphabetVerticalSize + 1));
        z = cat(2, z, z0_traj : z0_traj + alphabetVerticalSize);
    case 'A'
        % vertical line
        x = cat(2, x, x0_traj * ones(1, alphabetVerticalSize + 1));
        y = cat(2, y, y0_traj * ones(1, alphabetVerticalSize + 1));
        z = cat(2, z, z0_traj : z0_traj + alphabetVerticalSize);
        
        % horizontal line at the top
        x = cat(2, x, x0_traj : x0_traj + alphabetHorizontalSize);
        y = cat(2, y, y0_traj * ones(1, alphabetHorizontalSize + 1));
        z = cat(2, z, (z0_traj + alphabetVerticalSize) * ones(1, alphabetHorizontalSize + 1));
        
        % vertical line
        x = cat(2, x, (x0_traj + alphabetHorizontalSize) * ones(1, alphabetVerticalSize + 1));
        y = cat(2, y, y0_traj * ones(1, alphabetVerticalSize + 1));
        z = cat(2, z, z0_traj + alphabetVerticalSize : -1 : z0_traj);

        % move to center to draw the middle horizontal line
        x = cat(2, x, x(end));
        y = cat(2, y, y0_traj + cte);
        z = cat(2, z, z(end));

        % horizontal line
        x = cat(2, x, x0_traj : x0_traj + alphabetHorizontalSize);
        y = cat(2, y, y0_traj * ones(1, alphabetHorizontalSize + 1));
        z = cat(2, z, (z0_traj + alphabetVerticalSize / 2) * ones(1, alphabetHorizontalSize + 1));
end;

x = cat(2, x, x(end)) / aux;
y = cat(2, y, y0_traj + cte) / aux;
z = cat(2, z, z(end)) / aux;

for i = 1 : length(x)
    T(:, :, i) = [ 0  -1  0 x(i); 
                  0  0  -1 y(i); 
                   1  0  0 z(i); 
                   0  0   0   1];
end;

end

