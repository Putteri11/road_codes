function [ li ] = f_analyze_across_profs( sub_pc, sub_i_profs, dist, ...
    diff_z_std_multiplier, diff_z_th_multiplier, dist_across_profs )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

sub_n_pc = length(sub_pc(:,1));
sub_n_profs = max(sub_i_profs) - sub_i_profs(1) + 1;

rn = dist*1.1;
d = 3;
Q = sub_pc(1:1:sub_n_pc, 1:d);
q_inds = length(Q(:,1));

ins_neigh = f_find_neighbourhood(sub_pc, Q, rn, d);

first_prof = sub_i_profs(1);

diff_z_th_across_profs_array = zeros(sub_n_profs, 1);

for i_prof = first_prof:max(sub_i_profs)
    pc_prof = sub_pc(logical(sub_i_profs==i_prof), :);
    
    std_diff_z = std(diff(movmean(pc_prof(:, 3), 5)));
    diff_z_th = std_diff_z * diff_z_std_multiplier;
    
    diff_z_th_across_profs_array(i_prof - first_prof + 1) = ...
        diff_z_th * diff_z_th_multiplier;
end

prof_gap = round(dist_across_profs/dist);
neg_jump_inds = zeros(sub_n_pc, prof_gap);
pos_jump_inds = zeros(sub_n_pc, prof_gap);
neg_found = false;
neg_found_prof = 0;
pos_found = false;
found_jump_inds = false;

li_neg_jump = false(sub_n_pc, 1);
li_pos_jump = false(sub_n_pc, 1);

for i_q=1:q_inds-1
    ind = ins_neigh{i_q}(1);
    q_prof = sub_i_profs(ind);
    inds_this_prof = ins_neigh{i_q}(sub_i_profs(ins_neigh{i_q})==q_prof);
    inds_next_prof = ins_neigh{i_q}(sub_i_profs(ins_neigh{i_q})>q_prof);
    
    z = sub_pc(inds_this_prof, 3);
    z_next = sub_pc(inds_next_prof, 3);
    
    z_mean = mean(z);
    z_mean_next = mean(z_next);
    
    z_prof_diff = z_mean_next - z_mean;
    
    col_i = mod(q_prof-1, prof_gap) + 1;
    
    neg_prof_diff = q_prof - neg_found_prof;
    
    if neg_found && neg_prof_diff > prof_gap
        neg_found = false;
        pos_found = false;
        neg_jump_inds = zeros(sub_n_pc, prof_gap);
        pos_jump_inds = zeros(sub_n_pc, prof_gap);
    end

   
    if ~found_jump_inds
        diff_z_th_across_profs = diff_z_th_across_profs_array(q_prof - first_prof + 1);
    else
        diff_z_th_across_profs = diff_z_th_across_profs_array(q_prof - first_prof + 1)*3/4;
    end
    
    if z_prof_diff < -diff_z_th_across_profs
        neg_jump_inds(ind, col_i) = ind;
        neg_found = true;
        neg_found_prof = q_prof;
        found_jump_inds = true;
    elseif (z_prof_diff > diff_z_th_across_profs) && neg_found
        pos_jump_inds(ind, col_i) = ind;
        pos_found = true;
        found_jump_inds = true;
    else
        found_jump_inds = false;
    end
    
    if pos_found && sub_i_profs(ins_neigh{i_q+1}(1))==q_prof+1
        neg_col_i = mod(col_i - 2 + prof_gap, prof_gap) + 1;
        if sum(neg_jump_inds(:, neg_col_i))>0
            neg_jump_inds2 = reshape(neg_jump_inds(:, neg_col_i), [], 1);
            pos_jump_inds2 = reshape(pos_jump_inds(:, col_i), [], 1);
            neg_jump_inds2 = neg_jump_inds2(neg_jump_inds2>0);
            pos_jump_inds2 = pos_jump_inds2(pos_jump_inds2>0);
            neg_jump_inds3 = zeros(length(neg_jump_inds2), 1);
            pos_jump_inds3 = zeros(length(pos_jump_inds2), 1);
            flag = false;
            
            for neg_i = 1:length(neg_jump_inds2)
                for pos_i = 1:length(pos_jump_inds2)
                    neg_point = sub_pc(neg_jump_inds2(neg_i), 1:2);
                    pos_point = sub_pc(pos_jump_inds2(pos_i), 1:2);
                    d_test = sqrt(sum(diff(vertcat(neg_point, pos_point)).^2));
                    if d_test < dist_across_profs
                        pos_jump_inds3(pos_i) = pos_jump_inds2(pos_i);
                        flag = true;
                    end
                end
                if flag
                    neg_jump_inds3(neg_i) = neg_jump_inds2(neg_i);
                    flag = false;
                end
            end
            
            neg_jump_inds3 = neg_jump_inds3(neg_jump_inds3>0);
            pos_jump_inds3 = pos_jump_inds3(pos_jump_inds3>0);
            
            if abs(1 - length(pos_jump_inds3)/length(neg_jump_inds3)) < 0.5
                li_neg_jump(neg_jump_inds3) = true;
                li_pos_jump(pos_jump_inds3) = true;
            end
            
            if sum(neg_jump_inds(:, col_i))==0
                neg_found = false;
            end
            pos_found = false;
            neg_found_prof = 0;
            neg_jump_inds(:, neg_col_i) = zeros(sub_n_pc, 1);
            pos_jump_inds(:, col_i) = zeros(sub_n_pc, 1);
        end
    end
    
    if neg_prof_diff <= prof_gap && sub_i_profs(ins_neigh{i_q+1}(1))==q_prof+1
        neg_jump_inds(:, mod(col_i, prof_gap) + 1) = zeros(sub_n_pc, 1);
    end

end


li = li_neg_jump | li_pos_jump;

end

