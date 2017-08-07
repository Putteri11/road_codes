function [ ins_neigh ] = f_find_neighbourhood( sub_pc, li_cand, rn )
%Description...
%

% Dimension of the point cloud
d = 2;

% query points
Q = sub_pc(logical(li_cand), 1:d);

% create kdtree seach object
ns = createns(sub_pc(:, 1:d));

% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q, rn);

end