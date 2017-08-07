function [ li ] = f_neighbourhood_analysis( sub_pc, sub_i_profs, li_cand, ...
    ins_neigh, n_points_th )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n_pc = length(sub_pc(:,1));

q_indices = 1:sum(li_cand);

li_2 = false(n_pc, 1);

for i_q = q_indices
    search_profs = sub_i_profs(ins_neigh{i_q});
    q_prof = sub_i_profs(ins_neigh{i_q}(1));
    search_profs_unique = unique(search_profs);
    
    switch length(search_profs_unique)
        case 3
            other_profs = search_profs_unique([1,3]);
            inds1 = ins_neigh{i_q}(search_profs==other_profs(1));
            inds2 = ins_neigh{i_q}(search_profs==other_profs(2));
            if (sum(li_cand(inds1)) > n_points_th) && (sum(li_cand(inds2)) >= n_points_th)
                inds = ins_neigh{i_q}(search_profs~=q_prof);
                li_2(inds) = true;
            end
        case 2
            switch q_prof == search_profs_unique(2)
                case true
                    other_prof = search_profs_unique(1);
                otherwise
                    other_prof = search_profs_unique(2);
            end
            inds1 = ins_neigh{i_q}(search_profs==other_prof);
            if sum(li_cand(inds1)) > n_points_th
                inds = ins_neigh{i_q}(search_profs~=q_prof);
                li_2(inds) = true;
            end
        otherwise
            if length(search_profs_unique) ~= 1
                if sum(li_cand(ins_neigh{i_q})) >= n_points_th
                    li_2(ins_neigh{i_q}) = true;
                end
            end
    end
end

li = li_cand & li_2;

end

