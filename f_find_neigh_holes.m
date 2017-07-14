function [ li ] = f_find_neigh_holes( sub_pc, sub_i_profs, li_holes )
%Description...
%  

n_pc = length(sub_pc);

% Dimension of the point cloud
d = 2;

% Radius of the local neighbourhood
rn = 0.30; % (metres) 

% query points 
Q = sub_pc(logical(li_holes), 1:2);
q_indices = 1:length(Q(:,1));

% create kdtree seach object
ns = createns(sub_pc(:, 1:d));

% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q, rn);

li_2 = zeros(n_pc, 1);
n_points_th = 6;

for i_q = q_indices
    if (sum(li_holes(ins_neigh{i_q})) > n_points_th) && ...
            (li_2(ins_neigh{i_q}(1)) == 0)
        li_2(ins_neigh{i_q}) = 1;
    end
end


li = double(li_holes & li_2);

end