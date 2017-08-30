function [ inds, found_jump_inds, last_ind ] = f_analyze_prof( pc_prof, n_pc_prev, ...
    diff_z_std_multiplier, prof_range )
%f_analyze_prof attempts to find cracks or holes from a single profile.
%This is a helper function designed for f_find_cracks_and_holes, but can be
%used on its own.
%
%   Input: 
%       - pc_prof (l_prof x 5):
%           A single profile from the point cloud (sub_pc) of the road.
%       - n_pc_prev:
%           Number of points before this profile.
%       - diff_z_std_multiplier:
%           Parameter from f_find_cracks_and_holes, used for thresholding.
%       - prof_range (1 x [length from last_ind to the end of the profile]):
%           Actual indices of the profile for the part of the profile in
%           inspection.
%
%   Output:
%       - inds (l_prof x 1):
%            Point cloud (sub_pc) indices of positively classified points.
%            NOTE: actual length is very likely to be less than l_prof due
%            to preallocation of memory and the removal of zero values.
%       - on_top:
%           A flag to determine that some points were positively
%           classified.
%       - last_ind:
%           The index where this profile analysis was halted due to flag
%           (on_top) being set to true, or the last index of the profile.
%
%   Possible improvements:
%       - Instead of using a running average to filter out noise, take the
%       average of all the points within a small distance (~0.5cm?) or
%       within a very small time window (1-10 micro seconds?) of the
%       current point. Although this will probably increase the runtime of
%       the algorithm, this might help in ignoring the varying point
%       density of the road, which this function currently does not do at
%       all.
%
%   Author: Joona Savela 30.8.2017


diff_z = diff(movmean(pc_prof(:, 3), 5));
std_diff_z = std(diff_z);
% Set a threshold for classifying points.
diff_z_th = std_diff_z * diff_z_std_multiplier;

l_prof = length(pc_prof(:, 1));

neg_jump_inds = zeros(l_prof, 1); % Array that contains the negative jump indices.
pos_jump_inds = zeros(l_prof, 1); % Array that contains the positive jump indices.
inds = zeros(l_prof, 1); % Output preallocation, in case nothing is found. 
neg_found = false; % Flag for negative jumps found
pos_found = false; % Flag for positive jumps found
on_bottom = false; % Flag for neutral/flat points after negative points are found
on_top = false; % Flag for neutral/flat points after positive points are found
found_jump_inds = 0; % An integer from 0 to 2 to determine that some points
                     % of interest were found: 0 for nothing found, 1 for
                     % neg-pos groups found, and 2 for a gap in prof_range
                     % found.

last_ind = prof_range(end); % in case nothing is found

for i = 1:length(prof_range)
    ii = prof_range(i);
    index = n_pc_prev + ii; % index of the point cloud (sub_pc)
    
    % if missing indices are found in prof_range, break from the loop and
    % return such values that the algorithm checks the nearby sections of 
    % the next profile.
    if (i > 1) && (ii - prof_range(i - 1) > 1)
        found_jump_inds = 2;
        inds = [n_pc_prev + prof_range(i - 1), index]; 
        last_ind = ii;
        break;
    end
    
    % thresholding algorithm
    if (diff_z(ii) < -diff_z_th)
        % if on_bottom is true when new negative jump points were
        % detected, discard the old points
        if on_bottom
            neg_jump_inds = zeros(l_prof, 1);
            on_bottom = false;
        end
        neg_jump_inds(ii) = index;
        neg_found = true;
    elseif (diff_z(ii) > diff_z_th)
        if neg_found
            pos_jump_inds(ii) = index;
            pos_found = true;
        end
    else
        if pos_found
            on_top = true;
        elseif neg_found
            on_bottom = true;
        end
    end
    
    % if found neg and pos jump inds, redefine inds and last_ind, and break
    % from the loop
    if on_top
        found_jump_inds = 1;
        inds = union(neg_jump_inds(neg_jump_inds>0), ...
            pos_jump_inds(pos_jump_inds>0)); 
        last_ind = ii;
        break;
    end
end


end
