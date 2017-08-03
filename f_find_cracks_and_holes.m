function [ li ] = f_find_cracks_and_holes( sub_pc, sub_i_profs )
%f_find_cracks_and_holes finds the cracks and holes of a laser scanned road
% point cloud.
%   Input:
%       - Xyzti (n_pc x 5):
%           3D point cloud where the rows are sorted based on the time
%           stamps (fourth column) of the points. Each row corresponds to
%           a single point [x, y, z, t, i], where
%               x  is the x-coordinate of the point,
%               y  is the y-coordinate of the point,
%               z  is the z-coordinate of the point,
%               t  is the time stamp of the point, and
%               i  is the intensity of the point.
%           Note: Xyzti is assumed to contain only the points classified 
%           as points in the road in the original point cloud.
%       - ins_prof_pc (n_pc x 1):
%           Indices of the profiles for the point cloud Xyzti, i.e.
%           indicates in which profile each of the points belongs, gotten
%           with the function f_retrProfiles.
%   Output:
%       - li (n_pc x 1):
%           A vector consisting of either a double 0 or 1, where each
%           point/row i corresponds to the row i in the original point
%           cloud. The points with a value of 1 are points inside or
%           near a hole or a crack in the original point cloud.
%
%   Notes:
%       - At the moment, ...
%
%   TODO (delete this):
%


n_pc = length(sub_pc(:,1)); % number of points in point cloud
li_cand = zeros(n_pc, 1); % output preallocation 
first_prof = sub_i_profs(1);
n_profs = max(sub_i_profs) - first_prof + 1; % number of profiles

n_pc_profs = zeros(n_profs, 1); % number of points in each of the profiles,
% preallocation.
help_var = first_prof; % a helper variable

% constructing n_pc_profs (fast)
for ii=1:n_pc
    if sub_i_profs(ii) ~= help_var
        help_var = sub_i_profs(ii);
    end
    n_pc_profs(help_var - first_prof + 1) = n_pc_profs(help_var - first_prof + 1) + 1;
end


range_profs = 2:n_profs-1;

diff_z_th = 0.0023; % large 0.0028 (m); small 0.0023 (m)
window = 20;

n_pc_profs_cumsum = cumsum(n_pc_profs);

for i = range_profs
    prof_road = sub_pc(logical(i-1+first_prof==sub_i_profs), :);
    diff_z = diff(movmean(prof_road(:, 3), 5));
    l_prof = length(prof_road(:, 1));
    
    neg_jump_inds = zeros(l_prof, 1);
    pos_jump_inds = zeros(l_prof, 1);
    count = 0;
    in_window = false;
    
    for ii = 2:l_prof-1
        index = n_pc_profs_cumsum(i - 1) + ii;

        if diff_z(ii) < -diff_z_th
            neg_jump_inds(ii) = index;
            in_window = true;
        end
        
        if in_window && (diff_z(ii) > diff_z_th)
            pos_jump_inds(ii) = index;
        end

        if count >= window
            if any(neg_jump_inds > 0) && any(pos_jump_inds > 0)
                li_cand(neg_jump_inds(neg_jump_inds > 0)) = true;
                li_cand(pos_jump_inds(pos_jump_inds > 0)) = true;
            end
            % reset state
            neg_jump_inds = zeros(l_prof, 1);
            pos_jump_inds = zeros(l_prof, 1);
            count = 0;
            in_window = false;
        end
        if in_window
            count = count + 1;
        end

    end
end

% neighbourhood analysis
rn = 0.06; % radius
li = f_neighbourhood_analysis(sub_pc, sub_i_profs, li_cand, rn);

end