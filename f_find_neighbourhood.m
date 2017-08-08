function [ ins_neigh ] = f_find_neighbourhood( sub_pc, Q, rn )
%Description...
%

% Dimension of the point cloud
d = 2;

% query points
% if islogical(li_cand)
%     Q = sub_pc(logical(li_cand), 1:d);
% else
%     Q = sub_pc(li_cand, 1:d);
% end

% create kdtree seach object
ns = createns(sub_pc(:, 1:d));

% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q(:, 1:d), rn);

end