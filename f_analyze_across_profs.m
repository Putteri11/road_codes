function [ li ] = f_analyze_across_profs( sub_pc, sub_i_profs, n_pc_profs_cumsum, ...
    dist, diff_z_std_multiplier, diff_z_th_multiplier, dist_across_profs, ...
    f_mirror, timestamp_th, dist_th )
%f_analyze_across_profs classifies defects across the profiles of the point
%cloud.
%
%   Input:
%       - sub_pc (sub_n_pc x 5):
%           point cloud of the road
%       - sub_i_profs (sub_n_pc x 1):
%           profile indices of sub_pc from f_retrProfiles
%       - n_pc_profs_cumsum (sub_n_profs x 1):
%           cumulative sum of the number of points in the profiles
%       - dist:
%           average (minimum) distance between profiles
%       - diff_z_std_multiplier:
%           parameter from f_find_cracks_and_holes
%       - diff_z_th_multiplier:
%           parameter from f_find_cracks_and_holes
%       - dist_across_profs:
%           parameter from f_find_cracks_and_holes
%       - f_mirror:
%           scanner mirror frequency
%       - timestamp_th:
%           parameter from f_find_cracks_and_holes
%       - dist_th:
%           parameter from f_find_cracks_and_holes
%
%   Output:
%       - li (sub_n_pc x 1):
%           logical indices of classified candidate points for the input
%           point cloud (sub_pc)
%
%   Possible improvements:
%       - Concatenating the input parameters (from f_find_crakcs_and_holes)
%       into a single input array, then extract/name the parameters inside
%       the function. I.e. reduce the number of inputs (currently 10)!
%       - Efficiency and/or algorithm improvements
%
%   Author: Joona Savela 30.8.2017


% Initializations
sub_n_pc = length(sub_pc(:,1));
first_prof = sub_i_profs(1);
sub_n_profs = max(sub_i_profs) - first_prof + 1;

% Create an array that stores the standard deviations of the difference of
% the running average of the z coordinate for each of the profiles. Used
% for calculating the threshold for each of the profiles.
diff_z_th_across_profs_array = zeros(sub_n_profs, 1);

for i_prof = first_prof:max(sub_i_profs)
    pc_prof = sub_pc(logical(sub_i_profs==i_prof), :);
    
    std_diff_z = std(diff(movmean(pc_prof(:, 3), 5)));
    diff_z_th = std_diff_z * diff_z_std_multiplier;
    
    diff_z_th_across_profs_array(i_prof - first_prof + 1) = ...
        diff_z_th * diff_z_th_multiplier;
end

% Number of profiles taken into consideration when extracting candidate points
prof_gap = round(dist_across_profs/dist); 

% Placeholders and flags used for classifying candidate points
neg_jump_inds = zeros(sub_n_pc, prof_gap);
pos_jump_inds = zeros(sub_n_pc, prof_gap);
neg_found = false;
neg_found_prof = 0;
pos_found = false;
found_jump_inds = false;

li_neg_jump = false(sub_n_pc, 1); % Half of the output, preallocation
li_pos_jump = false(sub_n_pc, 1); % Half of the output, preallocation

for i=2:sub_n_profs-1 % skip the first and the last profile
    prof_i = i-1+first_prof;
    pc_prof = sub_pc(logical(sub_i_profs==prof_i), :); % current profile
    l_prof = length(pc_prof(:,1));
    next_pc_prof = sub_pc(logical(sub_i_profs==prof_i+1), :); % next profile
    
    col_i = mod(prof_i-1, prof_gap) + 1; % column of neg_jump_inds and pos_jump_inds
    
    neg_prof_diff = prof_i - neg_found_prof;
    
    % if out of range of prof_gap, clear variables
    if neg_found && neg_prof_diff > prof_gap
        neg_found = false;
        pos_found = false;
        neg_jump_inds = zeros(sub_n_pc, prof_gap);
        pos_jump_inds = zeros(sub_n_pc, prof_gap);
    end
    
    for ii = 1:l_prof
        ind = n_pc_profs_cumsum(i-1)+ii; % index in the point cloud
        
        % Calculate the difference between the average z coordinates in
        % this and the next profile
        
        
        li_this_prof = abs( pc_prof(:,4)-pc_prof(ii,4) ) < timestamp_th;
        li_next_prof = abs( (next_pc_prof(:,4)-pc_prof(ii,4))-1/f_mirror ) ...
            < timestamp_th;
        
        z = pc_prof(li_this_prof, 3);
        z_next = next_pc_prof(li_next_prof, 3);
        
        z_mean = sum(z)/length(z); % Slightly faster than MATLAB's "mean" function
        z_mean_next = sum(z_next)/length(z_next);
        
        z_prof_diff = z_mean_next - z_mean;
        
        % make threshold smaller if jump inds were found recently
        if ~found_jump_inds
            diff_z_th_across_profs = diff_z_th_across_profs_array(prof_i - first_prof + 1);
        else
            diff_z_th_across_profs = diff_z_th_across_profs_array(prof_i - first_prof + 1)*3/4;
        end
        
        % Check whether the point is a negative or a positive jump
        if z_prof_diff < -diff_z_th_across_profs
            neg_jump_inds(ind, col_i) = ind;
            neg_found = true;
            neg_found_prof = prof_i;
            found_jump_inds = true;
        elseif (z_prof_diff > diff_z_th_across_profs) && neg_found
            pos_jump_inds(ind, col_i) = ind;
            pos_found = true;
            found_jump_inds = true;
        else
            found_jump_inds = false;
        end
        
    end
    
    % This part of the algorithm could be improved
    if pos_found        
        % only take negative jumps from the previous profile into
        % consideration
        neg_col_i = mod(col_i - 2 + prof_gap, prof_gap) + 1;
        if sum(neg_jump_inds(:, neg_col_i))>0
            % make new arrays for classified candidate indices
            neg_jump_inds2 = neg_jump_inds(:, neg_col_i);
            pos_jump_inds2 = pos_jump_inds(:, col_i);
            neg_jump_inds2 = neg_jump_inds2(neg_jump_inds2>0);
            pos_jump_inds2 = pos_jump_inds2(pos_jump_inds2>0);
            neg_jump_inds3 = zeros(length(neg_jump_inds2), 1);
            pos_jump_inds3 = zeros(length(pos_jump_inds2), 1);
            flag = false;
            
            % Check that the points (neg to pos) are close to each other
            for neg_i = 1:length(neg_jump_inds2)
                for pos_i = 1:length(pos_jump_inds2)
                    neg_point = sub_pc(neg_jump_inds2(neg_i), 1:2);
                    pos_point = sub_pc(pos_jump_inds2(pos_i), 1:2);
                    d_test = sqrt(sum(diff(vertcat(neg_point, pos_point)).^2));
                    if d_test < dist_across_profs
                        pos_jump_inds3(pos_i) = pos_jump_inds2(pos_i);
                        flag = true;
                    end
                end
                if flag
                    neg_jump_inds3(neg_i) = neg_jump_inds2(neg_i);
                    flag = false;
                end
            end
            
            neg_jump_inds3 = neg_jump_inds3(neg_jump_inds3>0);
            pos_jump_inds3 = pos_jump_inds3(pos_jump_inds3>0);
            
            % Divide the found neg candidates into groups of subsequent
            % points, and check that their physical 2D length is above a
            % threshold
            if ~isempty(neg_jump_inds3)
                
                % End indices of the groups
                neg_group_end_inds = vertcat(...
                    find(diff(diff(neg_jump_inds3))<0), length(neg_jump_inds3));
                
                % Start index of the groups, gets updated within the loop
                % below
                neg_group_start_ind = 1;
                
                for i_end_inds = 1:length(neg_group_end_inds)
                    % Physical 2D distance between the start and the end of
                    % a group
                    test_dist = sqrt(sum(diff(vertcat(...
                        sub_pc(neg_jump_inds3(neg_group_start_ind), 1:2), ...
                        sub_pc(neg_jump_inds3(neg_group_end_inds(i_end_inds)), 1:2))).^2));
                    
                    % Update the start index
                    neg_group_start_ind = neg_group_end_inds(i_end_inds) + 1;
                    
                    % Apply threshold and classify point
                    if test_dist > dist_th
                        li_neg_jump(neg_jump_inds3) = true;
                    end
                end
            end
            
            % The same for positive jumps
            if ~isempty(pos_jump_inds3)
                pos_group_end_inds = vertcat(...
                    find(diff(diff(pos_jump_inds3))<0), length(pos_jump_inds3));
                pos_group_start_ind = 1;
                for i_end_inds = 1:length(pos_group_end_inds)
                    test_dist = sqrt(sum(diff(vertcat(...
                        sub_pc(pos_jump_inds3(pos_group_start_ind), 1:2), ...
                        sub_pc(pos_jump_inds3(pos_group_end_inds(i_end_inds)), 1:2))).^2));
                    pos_group_start_ind = pos_group_end_inds(i_end_inds) + 1;
                    if test_dist > dist_th
                        li_pos_jump(pos_jump_inds3) = true;
                    end
                end
            end
            
            % If there are negative jump groups in this profile, leave
            % neg_found as true. This helps with cracks that are nearly
            % perpindicular to the road but not exactly.
            if sum(neg_jump_inds(:, col_i))==0
                neg_found = false;
            end
            % reset other variables
            pos_found = false;
            neg_found_prof = 0;
            neg_jump_inds(:, neg_col_i) = zeros(sub_n_pc, 1);
            pos_jump_inds(:, col_i) = zeros(sub_n_pc, 1);
        end
    end
    
    % Reset the least recently ("oldest") used column of neg_jump_inds to zeros
    if neg_prof_diff <= prof_gap
        neg_jump_inds(:, mod(col_i, prof_gap) + 1) = zeros(sub_n_pc, 1);
    end
    
end

% combine the found candidate groups
li = li_neg_jump | li_pos_jump;

end

