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
li_cracks = zeros(n_pc, 1); % output preallocation 
li_holes = zeros(n_pc, 1); % output preallocation 
first_prof = sub_i_profs(1);
n_profs = max(sub_i_profs) - first_prof + 1; % number of profiles

n_pc_profs = zeros(n_profs, 1); % number of points in all the profiles,
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


% classification parameters
% i_th = mean(Xyzti(:,5))/1.3; % threshold for the intensity
% grad_z_th = 0; % threshold for the difference in the gradient of z
% d_th = 3; % threshold for the fraction of the distances

% monte carlo 1
% i_th = 2092.2616;
% grad_z_th = 0.0013997;
% d_th = 3.49;

i_th_hole = 1647.613;
grad_z_th_hole = 0.00042294;
d_th_hole = 3.508;

i_th_crack = 3000;
grad_z_th_crack = 0.005;
d_th_crack = 1.5;

n_pc_profs_cumsum = cumsum(n_pc_profs);

for i = range_profs
    prof_road = sub_pc(logical(i-1+first_prof==sub_i_profs), :);
    grad_z = gradient(prof_road(:, 3));
    l_prof = length(prof_road(:, 1));
    for ii = 2:l_prof-1
        dist1 = sqrt( (prof_road(ii, 1) - prof_road(ii-1, 1))^2 + ...
            (prof_road(ii, 2) - prof_road(ii-1, 2))^2);
        dist2 = sqrt( (prof_road(ii, 1) - prof_road(ii+1, 1))^2 + ...
            (prof_road(ii, 2) - prof_road(ii+1, 2))^2);
        dist_frac = dist1 / dist2;
        is_hole_d = max(dist_frac, 1/dist_frac) > d_th_hole;
        is_crack_d = max(dist_frac, 1/dist_frac) > d_th_crack;
        
        intensity = prof_road(ii, 5);
        is_hole_i = intensity < i_th_hole;
        is_crack_i = intensity < i_th_crack;
        
        abs_grad_z = abs(grad_z(ii));
        is_hole_grad_z = abs_grad_z > grad_z_th_hole;
        is_crack_grad_z = abs_grad_z > grad_z_th_crack;
        
        is_hole = is_hole_d && is_hole_i && is_hole_grad_z;
        is_crack = is_crack_d && is_crack_i && is_crack_grad_z;
        
        if is_hole
            index = n_pc_profs_cumsum(i - 1) + ii;
            li_holes(index) = 1;
        end
        if is_crack
            index = n_pc_profs_cumsum(i - 1) + ii;
            li_cracks(index) = 1;
        end
    end
end

% neighbourhood analysis
li = double(f_find_neigh_cracks(sub_pc, sub_i_profs, li_cracks) | ...
        f_find_neigh_holes(sub_pc, sub_i_profs, li_holes));




end