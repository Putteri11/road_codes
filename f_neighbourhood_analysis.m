function [ li ] = f_neighbourhood_analysis( sub_pc, sub_i_profs, li_cand, rn )
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
n_points_th = 2;

for i_q = q_indices
    search_profs = unique(sub_i_profs(ins_neigh{i_q}));
    assert(length(search_profs)==3);
%     q_prof = search_profs(2);
    search_profs = search_profs([1,3]);
    inds1 = ins_neigh{i_q}(sub_i_profs(ins_neigh{i_q})==search_profs(1));
    inds2 = ins_neigh{i_q}(sub_i_profs(ins_neigh{i_q})==search_profs(2));
    if (sum(li_cand(inds1)) > n_points_th) || (sum(li_cand(inds2)) > n_points_th)
        li_2(inds1) = true;
    end
end


li = li_cand & li_2;

end