function [ li ] = f_find_neigh_cracks( sub_pc, li_cracks )
%Description...
%   

n_pc = length(sub_pc);
li_1 = zeros(n_pc, 1);

% 1st phase
li_cracks_indices = find(li_cracks == 1, n_pc);
neighbouring_indices = find(diff(li_cracks_indices) == 1, length(li_cracks_indices));

li_1(li_cracks_indices(neighbouring_indices)) = 1;
li_1(li_cracks_indices(neighbouring_indices) + 1) = 1;

% 2nd phase
% Dimension of the point cloud
d = 2;

% Radius of the local neighbourhood
rn = 0.2; % (metres) 

% query points 
Q = sub_pc(logical(li_1), 1:2);
q_indices = 1:length(Q(:,1));

% create kdtree seach object
ns = createns(sub_pc(:, 1:d));

% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q, rn);

li_2 = zeros(n_pc, 1);
n_points_th = 8;

for i_q = q_indices
    if (sum(li_1(ins_neigh{i_q})) > n_points_th) && ...
            (li_2(ins_neigh{i_q}(1)) == 0)
        li_2(ins_neigh{i_q}) = 1;
    end
end


li = double(li_1 & li_2);
end