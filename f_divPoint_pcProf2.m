function ind_profDivPoint = f_divPoint_pcProf2(Xyzt_fc, Traj)
%
% [outputVar_1, outputVar_2] = f_functionName(inputVar_1, inputVar_2) 
%
% Same as f_divPoint_pcProf, but uses a different criterion to find the
% dividing point. Now, the point that is furthest from the traj
% horizontally is chosen as the dividing point.
% 
% Estimate a point where a profile starts (ends) in an MLS point cloud.
%
%
% ------
% Input: 
%
%   Xyzt_fc (n_pts_prof x 4)    n_pts_prof consecutive points of a point
%                               cloud (based on time stamps) which
%       constitute a full circle of the scanner mirror. Each row
%       corresponds to one point and equals [x, y, z, t], where
%           x   is the x-coordinate of the point,
%           y   is the y-coordinate of the point,
%           z   is the z-coordinate of the point and
%           t   is the time stamp of the point. 
% 
%   Traj (n_traj x 4)   The trajectory of the scanner in which the rows 
%                   are sorted based on the times stamps of the trajectory
%                   points, which are in the fourth column. The time stamps
%                   correspond to the time stamps of the input point cloud
%                   (they are in the same "time zone"). Each row
%                   contains the laser scanning system origing at a
%                   time t_i and equals [x_i, y_i, z_i, t_i], where
%                       x_i   is the x-coordinate of the trajectory at time t_i,
%                       y_i   is the y-coordinate of the trajectory at time t_i,
%                       z_i   is the z-coordinate of the trajectory at time t_i and
%                       t_i   is the time stamp of the trajectory.
%
% -------
% Output:
%
%   outputVar_1 (n x m) Explanation.
%   outputVar_2 (n x m) cell array. Explanation.
% 
% 
% ----------------------------------------------------
% Calls functions (children, grandchildren with tabs):
% 
%   f_childFunction1.m
% 
% -------------------------------------------------
% Current version dd.mm.yyyy, (c) M.Lehtomäki, yyyy
%
% Remarks:
%   -   
%
% 
% -------------------
% Previous versions:
% 
% -------------------------------------------------
% Version dd.mm.yyyy, (c) M.Lehtomäki, yyyy
%
% Updates to previous versions:
%   -   
% 
% Remarks:
%   -   
%
%
%

%% Initialisations

% Column indices of the input trajectory matrix
ci_Traj_x = 1; % x-coordinates
ci_Traj_y = 2; % y-coordinates
ci_Traj_z = 3; % z-coordinates
ci_Traj_t = 4; % time stamps

%% Find trajectory locations for the points in the full circle

% For the middle point in FFC, find the closest point in the trajectory
% based on time stamps
ind_closestTrajPoint = f_closestTrajPoint(Xyzt_fc(round(size(Xyzt_fc, 1)/2), 4), Traj(:, ci_Traj_t));

% Use same trajectory location for each point in FFC
Traj_fc = f_repmat_mtx_fast(Traj(ind_closestTrajPoint, :), size(Xyzt_fc, 1), 1);

%% Find dividing point on the full circle of the scanner mirror

% Retrieve the point whose horizontal distance is the largest wrt traj. 

% ??this may not be perfect solution. If the first profile is not
% close to full, the dividing point may be close to the road. Later close
% to full profiles are then possibly divided from a stranged position. A
% better way would be to try Antero's idea where we check if the point is
% on the right or left side of the trajectory.

% Calculate vectors that point from the trajectory to the point cloud in
% the horizontal plane
V = Xyzt_fc(:, [1, 2]) - Traj_fc(:, [ci_Traj_x, ci_Traj_y]);

% Squared horizotal distances
d2_pc2traj = sum( V.^2, 2 ); % norms of the rows of V

% The index of the dividing point of the first profile
[~, ind_profDivPoint] = max( d2_pc2traj );

end

