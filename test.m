
ticID = tic;
n_pc_profs = zeros(n_profs, 1);
a = ins_prof_pc(1);

for ii=1:n_pc
    if ins_prof_pc(ii) ~= a
        a = ins_prof_pc(ii);
    end
    n_pc_profs(a) = n_pc_profs(a) + 1;
end % constructing counts

plot(n_pc_profs, 'b.')

t_elapsed_profRetr = toc(ticID);
disp(['Time elapsed: ', num2str(t_elapsed_profRetr), ' seconds.'])

% round(mean(counts))

% 
% %%
% 
% % [li_1, pc_1] = Carve_Several_2D_Objects_edit_Joona(Xyzti(1:10:end, 1:2), 3); % haetaan alueet harvemmasta pilvestä
% % li_2 = logical(Carve_Several_2D_Objects(Xyzti(:, 1:2), pc_1)); % haetaan alueissa olevat pisteet koko pilvestä
% 
% %%
% tic
% sub_pc = Xyzti(li_2, :);
% sub_n_pc = length(sub_pc);
% sub_i_profs = ins_prof_pc(li_2);
% sub_n_profs = max(sub_i_profs) - sub_i_profs(1) + 1;
% sub_counts = zeros(sub_n_profs, 1);
% aa = sub_i_profs(1);
% 
% for ii=1:sub_n_pc
%     if sub_i_profs(ii)==aa
%         sub_counts(aa - sub_i_profs(1) + 1) = sub_counts(aa - sub_i_profs(1) + 1) + 1;
%     else
%         aa = sub_i_profs(ii);
%         sub_counts(aa - sub_i_profs(1) + 1) = sub_counts(aa - sub_i_profs(1) + 1) + 1;
%     end
% end
% 
% % monte carlo 
% % i_th = 1600 + 50*randn;
% % grad_z_th = 0.0004 + 0.0001*randn;
% % d_th = 3.0 + 0.2*randn;
% % percentage = 0.4 + 0.05*randn;
% 
% 
% % fixed values
% i_th = 10000;
% grad_z_th = 0.005;
% d_th = 1.5;
% 
% test_li = zeros(sub_n_pc, 1);
% 
% for i=2:sub_n_profs-1
%     prof_road = sub_pc(logical(i-1+sub_i_profs(1)==sub_i_profs), :);
%     grad_z = gradient(prof_road(:, 3));
%     l_prof = length(prof_road(:, 1));
%     for ii = 2:l_prof-1
%         dist1 = sqrt( (prof_road(ii, 1) - prof_road(ii-1, 1))^2 + ...
%             (prof_road(ii, 2) - prof_road(ii-1, 2))^2);
%         dist2 = sqrt( (prof_road(ii, 1) - prof_road(ii+1, 1))^2 + ...
%             (prof_road(ii, 2) - prof_road(ii+1, 2))^2);
%         dist_frac = dist1 / dist2;
%         isDefect_d = dist_frac > d_th || dist_frac < 1/d_th;
%         
%         intensity = prof_road(ii, 5);
%         isDefect_i = intensity < i_th;
%         
%         abs_grad_z = abs(grad_z(ii));
%         isDefect_grad_z = abs_grad_z > grad_z_th;
%         
%         isDefect = isDefect_d && isDefect_i && isDefect_grad_z;
%         
%         if isDefect
%             helper = cumsum(sub_counts);
%             index = helper(i - 1) + ii;
%             test_li(index) = 1;
%         end
%     end
% end
% 
% 
% test_li_indices = find(test_li == 1, sub_n_pc);
% neighbouring_indices = find(diff(test_li_indices) == 1, length(test_li_indices));
% 
% out_li = zeros(sub_n_pc, 1);
% out_li(test_li_indices(neighbouring_indices)) = 1;
% out_li(test_li_indices(neighbouring_indices) + 1) = 1;
% 
% out_li = logical(out_li);
% 
% % out_li = logical(test_li);
% 
% toc
% 
% f_initFig(1, 'w');
% % plot(Xyzti(:, 1), Xyzti(:, 2), '.', 'markersize', 2, 'color', 0.7*[1 1 1])
% % plot(Xyzti(li_2, 1), Xyzti(li_2, 2), 'o', 'markersize', 6)
% fscatter3_edit_Joona(sub_pc(:, 1), sub_pc(:, 2), sub_pc(:, 3), sub_pc(:, 5), cmap);
% plot3(sub_pc(out_li, 1), sub_pc(out_li, 2), sub_pc(out_li, 3), 'ro', 'markersize', 6);
% 
% out_text = ['i_th: ', num2str(i_th), '; ', ...
%     'grad_z_th: ', num2str(grad_z_th), '; ', 'd_th: ', num2str(d_th)];
% 
% disp(out_text);
% 
% 
% %%
% 
% f_initFig(2, 'w');
% li_3 = logical(sub_i_profs == 16 + sub_i_profs(1) - 1);
% fscatter3(sub_pc(li_3, 1), sub_pc(li_3, 2), sub_pc(li_3, 3), sub_pc(li_3, 5), cmap)
% 
% 
% %%
% ticID = tic;
% 
% [sub_pc, ~] = f_find_road_raw(Xyzti, ins_prof_pc);
% 
% n_skip = 10;
% 
% close all;
% f_initFig(1, 'k')
% fscatter3_edit_Joona(sub_pc(1:n_skip:end, 1), sub_pc(1:n_skip:end, 2), sub_pc(1:n_skip:end, 3), sub_pc(1:n_skip:end, 5), cmap);
% 
% t_elapsed_profRetr = toc(ticID);
% disp(['Time elapsed: ', num2str(t_elapsed_profRetr), ' seconds.'])

%%

Xyzi = Xyzti(:, [1,2,3,5]);

cumsum_n_pc_profs = cumsum(n_pc_profs);

[sub_pc, sub_i_profs] = f_find_road_raw(Xyzi, ins_prof_pc);

i_prof = 2302;

Xyzi_prof = sub_pc(logical(sub_i_profs==i_prof), :);

cand = f_roadEdgeCandPts(Xyzi_prof, par_rse);

% disp(cand);

%%
li_test = cand.li_ptsProf_posHeightJump;
disp(sum(li_test));
% indices = 1:n_pc;
% mid_i = indices(logical(sub_i_profs==i_prof));
mid_i = cumsum_n_pc_profs(i_prof - 1);
offset = 5e5;
start_i = mid_i - offset;
end_i = mid_i + offset;

n_skip = 10;
ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'k')
fscatter3_edit_Joona(Xyzti(start_i:n_skip:end_i, 1), Xyzti(start_i:n_skip:end_i, 2), Xyzti(start_i:n_skip:end_i, 3), Xyzti(start_i:n_skip:end_i, 5), cmap);
plot3(Xyzi_prof(li_test, 1), Xyzi_prof(li_test, 2), Xyzi_prof(li_test, 3), 'wo', 'linewidth', 1, 'markersize', 6);




