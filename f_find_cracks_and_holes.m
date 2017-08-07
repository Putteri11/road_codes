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

diff_z_th = 0.001; % large 0.0028 (m); small 0.0023 (m); smallest cracks: ~0.001 (m)
window = 20;
jump_diff_th = 5;

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
            count = 0;
        end
        
        if in_window && (diff_z(ii) > diff_z_th)
            pos_jump_inds(ii) = index;
        end

        if count >= window
            if any(neg_jump_inds > 0) && any(pos_jump_inds > 0)
                if abs(sum(neg_jump_inds>0) - sum(pos_jump_inds>0)) < jump_diff_th
                    li_cand(neg_jump_inds(neg_jump_inds > 0)) = true;
                    li_cand(pos_jump_inds(pos_jump_inds > 0)) = true;
                end
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


% Find an approximate distance between profiles
n_pc_range = 1:round(mean(n_pc_profs)*2);
sub_sub_pc = sub_pc(n_pc_range, :);
sub_sub_i_profs = sub_i_profs(n_pc_range);

dist_arr = zeros(n_pc_range(end), 1);
prof_range = min(sub_sub_i_profs):max(sub_sub_i_profs)-1;

for i_prof = prof_range
    src_pts = sub_sub_pc(sub_sub_i_profs==i_prof, 1:3);
    dst_pts = sub_sub_pc(sub_sub_i_profs==i_prof+1, 1:3);
    
    for i_src = 1:10:length(src_pts(:,1))
        % binary search
        src = src_pts(i_src, :);
        left = 1;
        right = length(dst_pts(:,1));
        flag = 0;
        
        while left <= right - 2
            mid = ceil((left + right) / 2);
            d_1 = sqrt(sum(diff(vertcat(src, dst_pts(mid-1, :))).^2));
            d_2 = sqrt(sum(diff(vertcat(src, dst_pts(mid, :))).^2));
            d_3 = sqrt(sum(diff(vertcat(src, dst_pts(mid+1, :))).^2));
            
            if (d_2 <= d_1) && (d_2 <= d_3)
                min_cand = d_2;
                flag = 1;
                break;
            elseif (d_2 > d_3) || (d_1 > d_2)
                left = mid + 1;
            else
                right = mid - 1;
            end
        end    
        
        if flag == 0
            min_cand = min([d_1, d_2, d_3]);
        end
        
        dist_arr(n_pc_profs_cumsum(i_prof-prof_range(1)+1) + i_src) = min_cand;
    end
end

dist_arr = dist_arr(dist_arr>0);
dist_arr = dist_arr(abs(diff(dist_arr))<0.02);

dist = mean(dist_arr);

% neighbourhood analysis
rn = dist*1.15; % radius
n_points_th = 1;
ins_neigh = f_find_neighbourhood(sub_pc, li_cand, rn);
li = f_neighbourhood_analysis(sub_pc, sub_i_profs, li_cand, ins_neigh, n_points_th);

rn = dist*4;
n_points_th = 20;
ins_neigh = f_find_neighbourhood(sub_pc, li, rn);
li = f_neighbourhood_analysis(sub_pc, sub_i_profs, li, ins_neigh, n_points_th);

% li = li_cand;
end