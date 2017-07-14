function [ins_prof_pc, ins_pc_profStart, ins_pc_profStart_reset, ins_prof_reset] = ...
    f_retrProfiles(Xyzt, Traj, f_mirror)
%
% [ins_prof_pc, ins_pc_profStart, ins_pc_profStart_reset, ins_prof_reset] = ...
%     f_retrProfiles(Xyzt, Traj, f_mirror)
%
% Retrieve profiles from a laser scanner point cloud. In this case, the
% profile means the scan line along the rotation of the laser scanner, that
% is, along the curve that the adjacent points form on surfaces.
%
%
% ------
% Input:
%
%   Xyzt (n_pc x 4) 3D Point cloud in which the rows are sorted based on
%                   the times stamps of the points, which are in the fourth
%                   column. Each row corresponds to one point and
%                   equals [x, y, z, t], where
%                       x   is the x-coordinate of the point,
%                       y   is the y-coordinate of the point,
%                       z   is the z-coordinate of the point and
%                       t   is the time stamp of the point.
%
%   Traj (n_traj x 4)    The trajectory of the scanner in which the rows
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
%   f_mirror  (Hz)        Mirror frequency of the scanner.
%
% -------
% Output:
%
%   ins_prof_pc    (n_pc x 1) For each point (row) in the input point
%                   cloud matrix, the index of the profile it belongs to.
%
%
% -------------------------------------------------
% Current version 16.11.2016, (c) M.Lehtomäki, 2016
%
% Remarks:
%   
%   -   ??remove extra testing output variables and comment them in the code
%   -   Tested in practice and works as it should (4.2..2017) Tested using
%       dataset "\VT1\160421_125641_VUX-1-HA_etrs.las 
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

% maximum number of points on one profile ??parametriksi inputtiin
n_pts_prof_max = 20000; % this corresponds to million points per second and 50 Hz mirror (20000 = 1e+6 / 50)

% Period of the scanning mirror
T_mirror = 1 / f_mirror;

% number of points in the point cloud and trajectory
n_pc = size(Xyzt, 1); % poin cloud
n_traj = size(Traj, 1); % trajectory

% Column indices of the input point cloud matrix
ci_Xyzt_x = 1; % x-coordinates
ci_Xyzt_y = 2; % y-coordinates
ci_Xyzt_z = 3; % z-coordinates
ci_Xyzt_t = 4; % time stamps

% Column indices of the input trajectory matrix
ci_Traj_x = ci_Xyzt_x; % x-coordinates
ci_Traj_y = ci_Xyzt_y; % y-coordinates
ci_Traj_z = ci_Xyzt_z; % z-coordinates
ci_Traj_t = ci_Xyzt_t; % time stamps


%% Find first full circle (FFC) of the mirror

% Retrieve all points collected during the time interval when the mirror
% has rotated the first full circle (FFC). That is, points whose time
% stamps are between t_1 and t_1 + T_mirror, where t_1 is the time stamp of
% the first point of the cloud.

t_1 = Xyzt(1, ci_Xyzt_t); % time stamp of the first point of the cloud
li_Xyzt_ffc = Xyzt(:, ci_Xyzt_t) >= t_1 & Xyzt(:, ci_Xyzt_t) < t_1 + T_mirror; % LI for the point cloud; true indicates that the point belongs to the FFC
P_ffc = Xyzt(li_Xyzt_ffc, :); % points in the FFC

%% First profile

% Division is based on the first dividing point and mirror period

ind_firstDivPoint = f_divPoint_pcProf2(P_ffc, Traj); % Find dividing point on the first full circle
t_end = Xyzt(end, ci_Xyzt_t); % time stamp of the last point of the cloud
ins_prof_pc = nan(n_pc, 1); % allocate memory for the profile indices
ind_prof_lastRetr = 0; % index of the last retrieved profile

% retrieve first, possibly non-full profile
li_Xyzt_prof = false(n_pc, 1); % Logical indices for rows (points) in Xyzt; true indicates that the point belongs to the current profile

if ind_firstDivPoint > 1 % the dividing point is not the first point
    li_Xyzt_prof(1 : ind_firstDivPoint) = true; % first profile in not necessarily full
    t_prof_end = Xyzt(ind_firstDivPoint, ci_Xyzt_t) + 1e-20; % end time of the first profile; add small number because t_prof_end should be larger than the time stamp of the last point
else % the dividing point is the first point of the cloud
    t_prof_start = t_1; % start time of the first profile
    t_prof_end = t_prof_start + T_mirror; % end time of the profile
    li_Xyzt_prof = Xyzt(:, ci_Xyzt_t) >= t_prof_start & Xyzt(:, ci_Xyzt_t) < t_prof_end; % Logical indices for points in Xyzt; true indicates that the point belongs to the current profile
end
ins_prof_pc(li_Xyzt_prof) = ind_prof_lastRetr + 1; % give profile indices for the points
ind_prof_lastRetr = ind_prof_lastRetr + 1; % Update profile index

%% Retrieve the rest profiles

% ??nopeutus: ota kerrallaan miljoona peräkkäistä pistettä. jäin tähän
% 26.1.2017

% testing variables
ins_pc_profStart = nan( 30000, 1 );
ins_pc_profStart_reset = nan( 30000, 1 );
ins_prof_reset = nan( 30000, 1 );
ind_startProfLast = 0;
ind_startProfLast_reset = 0;

% cumulative time between profiles retrieved with a same dividing point
tc = t_prof_end - t_1;

while t_prof_end <= t_end
    
    %% Retrieve profile points
    
    t_prof_start = t_prof_end; % start time of the profile
    t_prof_end = t_prof_start + T_mirror; % end time of the profile
    
    li_Xyzt_prof = Xyzt(:, ci_Xyzt_t) >= t_prof_start & Xyzt(:, ci_Xyzt_t) < t_prof_end; % Logical indices for points in Xyzt; true indicates that the point belongs to the current profile
    ins_prof_pc(li_Xyzt_prof) = ind_prof_lastRetr + 1; % give profile indices for the points
    ind_prof_lastRetr = ind_prof_lastRetr + 1; % Update profile index
    
    % update cumulative time without new dividing point
    tc = tc + t_prof_end - t_prof_start;
    
    %% Estimate dividing point every xx seconds
    
    if tc >= 0.3 % (seconds) ??lisää parametriksi inputtiin
        
        % new dividing point
        ins_pc_prof = find( li_Xyzt_prof ); % linear indices (pc) of the profile points
        ind_pc_divPoint = ins_pc_prof( f_divPoint_pcProf2(Xyzt(li_Xyzt_prof, :), Traj) ); % dividing point on the first full circle
        t_div = Xyzt( ind_pc_divPoint, 4 ); % time stamp of the dividing point
        
        % plotting
        %         ind_pc_profStart = find(li_Xyzt_prof, 1);
        %         ind_pc_profEnd = find(li_Xyzt_prof, 1, 'last');
        %         f_initFig(101, 'w')
        %         plot3(Xyzt(li_Xyzt_prof, 1), Xyzt(li_Xyzt_prof, 2), Xyzt(li_Xyzt_prof, 3), '.')
        %         plot3(Xyzt(ind_pc_divPoint, 1), Xyzt(ind_pc_divPoint, 2), Xyzt(ind_pc_divPoint, 3) , 'x', 'linewidth', 2, 'markersize', 20)
        %         plot3(Xyzt(ind_pc_profStart, 1), Xyzt(ind_pc_profStart, 2), Xyzt(ind_pc_profStart, 3) , 'diamond', 'linewidth', 2, 'markersize', 20)
        %         plot3(Xyzt(ind_pc_profEnd, 1), Xyzt(ind_pc_profEnd, 2), Xyzt(ind_pc_profEnd, 3) , 'square', 'linewidth', 2, 'markersize', 20)
        
        % points after and before new dividing point
        li_pc_beforeDiv = Xyzt(:, 4) >= t_prof_start & Xyzt(:, 4) < t_div;
        n_pts_before = sum(li_pc_beforeDiv);
        li_pc_afterDiv = Xyzt(:, 4) >= t_div & Xyzt(:, 4) < t_prof_end;
        n_pts_after = sum(li_pc_afterDiv);
        
        if n_pts_before >= n_pts_after
            
            % ??tsekkaa kaikki tässä ifin sisällä ja elsen sisällä
            % ??tsekkaa myös että ins_prof_pc ja ins_pc_prof eivät ole
            % menneet sekaisin
            
            % T_mirror is too large. Move points from the previous profile
            % to the current and from the current profile to the next. The
            % new dividing point is the end of the profile.
            t_prof_end = t_div; % end time of the profile
            t_prof_start = t_prof_end - T_mirror; % start time of the profile
            li_Xyzt_prof = Xyzt(:, ci_Xyzt_t) >= t_prof_start & Xyzt(:, ci_Xyzt_t) < t_prof_end; % Logical indices for points in Xyzt; true indicates that the point belongs to the current profile
            ins_prof_pc(li_Xyzt_prof) = ind_prof_lastRetr;
            
        else
            
            % T_mirror is too small. Move points from the next profile
            % to the current and from the current profile to the previous.
            % The new dividing point is the beginning of the profile.
            t_prof_start = t_div; % start time of the profile
            t_prof_end = t_prof_start + T_mirror; % end time of the profile
            li_Xyzt_prof = Xyzt(:, ci_Xyzt_t) >= t_prof_start & Xyzt(:, ci_Xyzt_t) < t_prof_end; % Logical indices for points in Xyzt; true indicates that the point belongs to the current profile
            ins_prof_pc(li_Xyzt_prof) = ind_prof_lastRetr;
            
            % Move points before the dividing point to the previous
            % profile
            ins_prof_pc( ins_pc_prof( ind_pc_divPoint : end ) ) = ind_prof_lastRetr - 1;
            
        end
        
        % set cumulative time to zero
        tc = 0; 
        
    end
    
    %% testing stuff
    
    ind_pc_profStart = find(li_Xyzt_prof, 1);
    ins_pc_profStart(ind_startProfLast + 1) = ind_pc_profStart;
    ind_startProfLast = ind_startProfLast + 1;
    % plot3(Xyzt(ind_pc_profStart, 1), Xyzt(ind_pc_profStart, 2), Xyzt(ind_pc_profStart, 3) , 'x', 'linewidth', 2, 'markersize', 20, 'color', [0, 0.45, 0.74]);
    % plot3(Xyzt(li_Xyzt_prof, 1), Xyzt(li_Xyzt_prof, 2), Xyzt(li_Xyzt_prof, 3), '.')
    
    if tc == 0
        ins_pc_profStart_reset(ind_startProfLast_reset + 1) = ind_pc_profStart;
        ins_prof_reset(ind_startProfLast_reset + 1) = ind_prof_lastRetr;
        ind_startProfLast_reset = ind_startProfLast_reset + 1;
        
    end
    
end

%% testing stuff

% f_initFig(101, 'w')
ins_pc_profStart = ins_pc_profStart(1:ind_startProfLast);
ins_pc_profStart_reset = ins_pc_profStart_reset(1:ind_startProfLast_reset);
ins_prof_reset = ins_prof_reset(1:ind_startProfLast_reset);
% colour1 = [0.93, 0.69, 0.13];
% colour2 = [0, 0.45, 0.74];
% plot3(Xyzt(ins_pc_profStart, 1), Xyzt(ins_pc_profStart, 2), Xyzt(ins_pc_profStart, 3) , 'x', 'linewidth', 2, 'markersize', 20, 'color', colour1);
% plot3(Xyzt(ins_pc_profStart_reset, 1), Xyzt(ins_pc_profStart_reset, 2), Xyzt(ins_pc_profStart_reset, 3) , 'o', 'linewidth', 2, 'markersize', 20, 'color', colour2);
% plot3(Xyzt(1:50:end, 1), Xyzt(1:50:end, 2), Xyzt(1:50:end, 3), '.', 'markersize', 2, 'color', 0.75*[1 1 1])





end

