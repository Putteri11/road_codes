function [ li ] = f_neighbourhood_analysis( sub_pc, sub_i_profs, li_cand, ...
    ins_neigh, n_points_th )
%f_neighbourhood_analysis filters out false positives by checking if
%there are enough positive candidate detections in the neighbourhood.
%   
%   Input:
%       - sub_pc (sub_n_pc x 5):
%           point cloud of the road
%       - sub_i_profs (sub_n_pc x 1):
%           profile indices of sub_pc from f_retrProfiles
%       - li_cand (sub_n_pc x 1):
%           logical indices of positively classified candidate points
%       - ins_neigh ([number of query points] x 1) (cell)
%           indices of found neihgbouring points, gotten with
%           f_find_neighbourhood
%       - n_points_th:
%           parameter from f_find_cracks_and_holes
%
%   Output:
%       - li (sub_n_pc x 1):
%           logical indices of classified points for the input point cloud
%           (sub_pc)
%   
%   Author: Joona Savela 28.8.2017

sub_n_pc = length(sub_pc(:,1));

n_q = sum(li_cand); % Number of query points

li_2 = false(sub_n_pc, 1); % Points that can be positively classified; preallocation

for i_q = 1:n_q
    found_points_inds = ins_neigh{i_q}; % All the found points' indices  
                                        % around a query point
    search_profs = sub_i_profs(found_points_inds); % Corresponding profile 
                                                   % indices
    q_prof = sub_i_profs(found_points_inds(1)); % Profile with the query point
    search_profs_unique = unique(search_profs); 
    
    % If the search radius is small, i.e. points are detected from 2 to 3
    % profiles, there must be enough points on all of the profiles except
    % the one with the query point.
    % Otherwise there must be enough points in the general area around the
    % query point.
    switch length(search_profs_unique)
        case 3
            other_profs = search_profs_unique([1,3]);
            inds1 = found_points_inds(search_profs==other_profs(1));
            inds2 = found_points_inds(search_profs==other_profs(2));
            % Notice the "and" operator
            if (sum(li_cand(inds1)) >= n_points_th) && (sum(li_cand(inds2)) >= n_points_th)
                li_2(found_points_inds) = true;
            end
        case 2
            switch q_prof == search_profs_unique(2)
                case true
                    other_prof = search_profs_unique(1);
                otherwise
                    other_prof = search_profs_unique(2);
            end
            inds1 = found_points_inds(search_profs==other_prof);
            if sum(li_cand(inds1)) >= n_points_th
                li_2(found_points_inds) = true;
            end
        otherwise
            if length(search_profs_unique) ~= 1
                if sum(li_cand(found_points_inds)) >= n_points_th
                    li_2(found_points_inds) = true;
                end
            end
    end
end

li = li_cand & li_2;

end

