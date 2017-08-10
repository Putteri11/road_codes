function [ inds, on_top, last_ind ] = f_analyze_prof( pc_prof, n_pc_prev, ...
    diff_z_std_multiplier, prof_range )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

diff_z = diff(movmean(pc_prof(:, 3), 5));
std_diff_z = std(diff_z);
diff_z_th = std_diff_z * diff_z_std_multiplier;

l_prof = length(pc_prof(:, 1));

neg_jump_inds = zeros(l_prof, 1);
pos_jump_inds = zeros(l_prof, 1);
inds = zeros(l_prof, 1);
neg_found = false;
pos_found = false;
on_bottom = false;
on_top = false;

last_ind = prof_range(end);

for ii = prof_range
    index = n_pc_prev + ii;
    
    if (diff_z(ii) < -diff_z_th)
        if on_bottom
            neg_jump_inds = zeros(l_prof, 1);
            on_bottom = false;
        end
        neg_jump_inds(ii) = index;
        neg_found = true;
    elseif (diff_z(ii) > diff_z_th)
        if neg_found
            pos_jump_inds(ii) = index;
            pos_found = true;
        end
    else
        if pos_found
            on_top = true;
        elseif neg_found
            on_bottom = true;
        end
    end
    
    if on_top
        inds = union(inds(inds>0), union(neg_jump_inds(neg_jump_inds>0), ...
            pos_jump_inds(pos_jump_inds>0)));
        last_ind = ii;
        break;
    end
end


end
