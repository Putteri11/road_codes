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
li_cand = false(n_pc, 1); % output preallocation
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

n_pc_profs_cumsum = cumsum(n_pc_profs);

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
        
        if flag == 0 && exist('mid', 'var')
            min_cand = min([d_1, d_3]);
        end
        
        if exist('min_cand', 'var')
            dist_arr(n_pc_profs_cumsum(i_prof-prof_range(1)+1) + i_src) = min_cand;
        end
    end
end

dist_arr = dist_arr(dist_arr>0);
dist_arr = dist_arr(abs(diff(dist_arr))<0.02);

dist = mean(dist_arr);

% apply thresholds
range_profs = 2:n_profs-1;

std_diff_z_th = 2.5; % large 0.0028 (m); small 0.0023 (m); smallest cracks: ~0.001 (m)

rn = dist*1.5;

for i = range_profs
    prof_i = i-1+first_prof;
    pc_prof = sub_pc(logical(prof_i==sub_i_profs), :);
    l_prof = length(pc_prof(:, 1));
    
    if l_prof > 0
        
        prof_range = 2:l_prof-1;
        last_ind = 1;
        
        while last_ind < prof_range(end)
            
            [jump_inds, found_jump_inds, last_ind] = f_analyze_prof(pc_prof, ...
                n_pc_profs_cumsum(i - 1), std_diff_z_th, prof_range);
            
            if found_jump_inds
                next_pc_prof = sub_pc(logical(prof_i+1==sub_i_profs), :);
                Q = pc_prof(jump_inds(1:end-1:end) - n_pc_profs_cumsum(i - 1), 1:3);
                ins_neigh_next_prof = f_find_neighbourhood(next_pc_prof, Q, rn);
                
                ins_next_prof_range = min(ins_neigh_next_prof{1}):max(ins_neigh_next_prof{2})-1;
                
                if ~isempty(ins_next_prof_range)
                    
                    [jump_inds_next_prof, found_jump_inds_next_prof, ~] = ...
                        f_analyze_prof(next_pc_prof, n_pc_profs_cumsum(i), ...
                        std_diff_z_th*3/4, ins_next_prof_range);
                    
                    if found_jump_inds_next_prof
                        li_cand(jump_inds) = true;
                        li_cand(jump_inds_next_prof) = true;
                    end
                end
            end
            
            prof_range = last_ind:prof_range(end);
            
        end
    end
end


% neighbourhood analysis
rn = dist*1.15; % radius
n_points_th = 1;
ins_neigh = f_find_neighbourhood(sub_pc, sub_pc(li_cand, 1:3), rn);
li = f_neighbourhood_analysis(sub_pc, sub_i_profs, li_cand, ins_neigh, n_points_th);

rn = dist*4;
n_points_th = 20;
ins_neigh = f_find_neighbourhood(sub_pc, sub_pc(li, 1:3), rn);
li = f_neighbourhood_analysis(sub_pc, sub_i_profs, li, ins_neigh, n_points_th);

% li = li_cand;
end