function [ ins_neigh ] = f_find_neighbourhood( sub_pc, Q, rn, d )
%f_find_neighbourhood finds all the points of the input point cloud that
%are within a given radius from the query points.
%   
%   Input:
%       - sub_pc (sub_n_pc x 5):
%           point cloud of the road, or a subsample of it
%       - Q ([number of query points] x d):
%           Query points
%       - rn:
%           search radius
%       - d:
%           search dimensions
%
%   Output:
%       - ins_neigh ([number of query points] x 1) (cell)
%           indices of found neihgbouring points
%
%   Author: Joona Savela 28.8.2017


% Dimension of the point cloud
if nargin<4
    d = 2;
end

% create kdtree seach object
ns = createns(sub_pc(:, 1:d));

% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q(:, 1:d), rn);

end