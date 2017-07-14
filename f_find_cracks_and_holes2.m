function [ li ] = f_find_cracks_and_holes2( Xyzti, ins_prof_pc )
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
%       - Extract road from pc (manually)
%       - Detect holes/hole edges:
%           - missing points
%           - low intensity?
%           - difference in z-coordinate
%           - distance in x-y-plane
%       - Label POIs with a double 1
%       - Document
%
%   Ideas (delete this):
%       - perhaps use machine learning to classify defects?
%           - manually label the POIs
%           - use a simple classifier
%           - would require quite a lot of data in order to work well
%       - test with both only the road, and the whole pc
%       - In a loop, remember the previous values of a profile, and do
%       classification based on them.
%           - try to match previous points as closely as possible.
%


n_pc = length(Xyzti(:,1)); % number of points in point cloud
li_1 = zeros(n_pc, 1); % output preallocation (li="logical index")
n_profs = max(ins_prof_pc); % number of profiles

n_pc_profs = zeros(n_profs, 1); % number of points in all the profiles;
% preallocation.
help_var = ins_prof_pc(1); % a helper variable

% constructing n_pc_profs (fast)
for ii=1:n_pc
    if ins_prof_pc(ii)==help_var
        n_pc_profs(help_var) = n_pc_profs(help_var) + 1;
    else
        help_var = ins_prof_pc(ii);
        n_pc_profs(help_var) = n_pc_profs(help_var) + 1;
    end
end

max_n_pc_profs = max(n_pc_profs);

% NOTE: the first and the last element of n_pc_profs is discarded because
% they're often small and hence incompatible
n_points = round(mean(n_pc_profs(2:end-1))*0.49); % number of points taken
% into consideration
ix_of_interest = zeros(n_profs, 2); % preallocation of the indices of interest
grad_z_th = 0.01; % threshold for the gradient of z
n_i_th = 200; % threshold for the number of indices in a sequence
start_i = n_pc_profs(1) + 1; % start index for Xyzti, for iterations in the loop
range_profs = 2:n_profs-1;

for i=range_profs
    end_i = start_i - 1 + n_pc_profs(i); % end index for Xyzti (for iterations
    % in the loop)
    try
        grad_z = gradient(Xyzti(start_i:end_i, 3));
        
        % finds all the indices of grad_z (i.e. indices of z) where its
        % value is below the threshold
        ix = find(abs(grad_z(1:end)) < grad_z_th, max_n_pc_profs);
        
        % finds all the starting indices of the sequences in ix in which the
        % values increment exactly by 1, at least n_i_th times
        ix2 = find(conv(double(diff(ix)==1), ones(1,n_i_th-1), 'valid')==n_i_th-1);
        
        first_i = ix(ix2(1))+30; % +30 for adjustment
        
        ix_of_interest(i, :) = [start_i + first_i, start_i + first_i + n_points - 1];
        
    catch
    end
    start_i = end_i + 1;
end



% classification parameters
% i_th = mean(Xyzti(:,5))/1.3; % threshold for the intensity
% grad_z_th = 0; % threshold for the difference in the gradient of z
% d_th = 3; % threshold for the fraction of the distances

% monte carlo 1
% i_th = 2092.2616;
% grad_z_th = 0.0013997;
% d_th = 3.49;

% monte carlo 2
i_th = 1647.613;
grad_z_th = 0.00042294;
d_th = 3.508;

% monte carlo 3
% i_th = 1981.8676;
% grad_z_th = 0.0023502;
% d_th = 4.7503;

% range_profs2 = range_profs(2:end); 


for i = range_profs
    ix1 = ix_of_interest(i,1);
    ix2 = ix_of_interest(i,2);
    % check for
    %   - low intensity?
    %   - difference in z-coordinate
    %   - distance in x-y-plane
    %     prof1_road = Xyzti(ix_of_interest(i_prof-1,1):ix_of_interest(i_prof-1,2), :);
    prof_road = Xyzti(ix1:ix2, :);
    grad_z = gradient(prof_road(:, 3));
    l_prof = length(prof_road(:, 1));
    for ii = 2:l_prof-1
        dist1 = sqrt( (prof_road(ii, 1) - prof_road(ii-1, 1))^2 + ...
            (prof_road(ii, 2) - prof_road(ii-1, 2))^2);
        dist2 = sqrt( (prof_road(ii, 1) - prof_road(ii+1, 1))^2 + ...
            (prof_road(ii, 2) - prof_road(ii+1, 2))^2);
        dist_frac = dist1 / dist2;
        isDefect_d = dist_frac > d_th || dist_frac < 1/d_th;
        
        intensity = prof_road(ii, 5);
        isDefect_i = intensity < i_th;
        
        abs_grad_z = abs(grad_z(ii));
        isDefect_grad_z = abs_grad_z > grad_z_th;
        
        isDefect = isDefect_d && isDefect_i && isDefect_grad_z;
        
        if isDefect
            index = ix1 - 1 + ii;
            li_1(index) = 1;
        end
    end
end

li_1_indices = find(li_1 == 1, n_pc);
neighbouring_indices = find(diff(li_1_indices) == 1, length(li_1_indices));

li = zeros(n_pc, 1);
li(li_1_indices(neighbouring_indices)) = 1;
li(li_1_indices(neighbouring_indices) + 1) = 1;


% for i=range_profs
%     li(ix_of_interest(i, 1):ix_of_interest(i, 2)) = 1;
% end


end