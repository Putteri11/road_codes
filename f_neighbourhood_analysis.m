function [ li ] = f_neighbourhood_analysis( sub_pc, li_cand, rn )
%Description...
%  

n_pc = length(sub_pc);

% Dimension of the point cloud
d = 2;

% query points 
Q = sub_pc(logical(li_cand), 1:2);
q_indices = 1:length(Q(:,1));

% create kdtree seach object
ns = createns(sub_pc(:, 1:d));

% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q, rn);

li_2 = false(n_pc, 1);
n_points_th = 10;

for i_q = q_indices
    if (sum(li_cand(ins_neigh{i_q})) > n_points_th) && ...
            (li_2(ins_neigh{i_q}(1)) == 0)
        li_2(ins_neigh{i_q}) = true;
    end
end


li = li_cand & li_2;

end