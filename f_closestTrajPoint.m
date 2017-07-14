function ind_closestTrajPoint = f_closestTrajPoint(t_query, t_traj)
%
% ind_closestTrajPoint = f_closestTrajPoint(t_query, t_traj)
%
% Find closest trajectory point for a query point with a time stamp
% t_query. Time stamps of the trajectory points are in t_traj.
%
%
% ------
% Input: 
%
%   inputVar_1 (n x m) Explanation.
%   inputVar_2 (n x m) cell array. Explanation.
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

%% Find closest trajectory point

% ??To get more accurate trajectory, use linear interpolation based on time
% stamps 

% For the middle point in FFC, find the closest point in the trajectory
% based on time stamps
ind_closestTrajPoint = f_match_1D(t_query, t_traj);


end

