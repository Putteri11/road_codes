
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
%%

[li_1, pc_1] = Carve_Several_2D_Objects_edit_Joona(Xyzti(1:10:end, 1:2), 3); % haetaan alueet harvemmasta pilvestä
li_2 = logical(Carve_Several_2D_Objects(Xyzti(:, 1:2), pc_1)); % haetaan alueissa olevat pisteet koko pilvestä

%%
tic
sub_pc = Xyzti(li_2, :);
sub_n_pc = length(sub_pc);
sub_i_profs = ins_prof_pc(li_2);
sub_n_profs = max(sub_i_profs) - sub_i_profs(1) + 1;
sub_counts = zeros(sub_n_profs, 1);
aa = sub_i_profs(1);

for ii=1:sub_n_pc
    if sub_i_profs(ii)==aa
        sub_counts(aa - sub_i_profs(1) + 1) = sub_counts(aa - sub_i_profs(1) + 1) + 1;
    else
        aa = sub_i_profs(ii);
        sub_counts(aa - sub_i_profs(1) + 1) = sub_counts(aa - sub_i_profs(1) + 1) + 1;
    end
end

% monte carlo
% i_th = 1600 + 50*randn;
% grad_z_th = 0.0004 + 0.0001*randn;
% d_th = 3.0 + 0.2*randn;
% percentage = 0.4 + 0.05*randn;


% fixed values
i_th = 2100;
diff_i_th = 130;
diff_z_th = 0.0023; % 0.0023/0.0028
diff_z_th2 = 0.0018;
window = 20;
% d_th = 0.028;

test_li = false(sub_n_pc, 1);
helper = cumsum(sub_counts);

for i=2:sub_n_profs-1
    prof_road = sub_pc(logical(i-1+sub_i_profs(1)==sub_i_profs), :);
    diff_z = diff(movmean(prof_road(:, 3), 5));
    intensity = movmean(prof_road(:, 5), 5);
    diff_i = diff(intensity);
    l_prof = length(prof_road(:, 1));
    
    neg_jump_inds = zeros(l_prof, 1);
    pos_jump_inds = zeros(l_prof, 1);
    count = 0;
    in_window = false;
    
    for ii = 2:l_prof-1
        index = helper(i - 1) + ii;
        
%         dist1 = sqrt( (prof_road(ii, 1) - prof_road(ii-1, 1))^2 + ...
%             (prof_road(ii, 2) - prof_road(ii-1, 2))^2 );
%         dist2 = sqrt( (prof_road(ii, 1) - prof_road(ii+1, 1))^2 + ...
%             (prof_road(ii, 2) - prof_road(ii+1, 2))^2 );
%         dist_frac = dist1 / dist2;
%         isDefect_d = dist_frac > d_th || dist_frac < 1/d_th;
%         isDefect_d = dist1 > d_th || dist2 > d_th;

%         isDefect_i = intensity(ii) < i_th && abs(grad_i(ii)) > grad_i_th;
% 
%         abs_grad_z = abs(grad_z(ii));
%         isDefect_grad_z1 = abs_grad_z > grad_z_th;
%         isDefect_grad_z2 = abs_grad_z > grad_z_th2;

%         isDefect = isDefect_grad_z1 || (isDefect_grad_z2 && isDefect_i);

        if diff_z(ii) < -diff_z_th
            neg_jump_inds(ii) = index;
            in_window = true;
        end
        
        if in_window && (diff_z(ii) > diff_z_th)
            pos_jump_inds(ii) = index;
        end

        if count >= window
            if any(neg_jump_inds > 0) && any(pos_jump_inds > 0)
                test_li(neg_jump_inds(neg_jump_inds > 0)) = true;
                test_li(pos_jump_inds(pos_jump_inds > 0)) = true;
            end
            neg_jump_inds = zeros(l_prof, 1);
            pos_jump_inds = zeros(l_prof, 1);
            count = 0;
            in_window = false;
        end
        if in_window
            count = count + 1;
        end
    end
end


% test_li_indices = find(test_li == 1, sub_n_pc);
% neighbouring_indices = find(diff(test_li_indices) == 1, length(test_li_indices));
% 
% out_li = zeros(sub_n_pc, 1);
% out_li(test_li_indices(neighbouring_indices)) = 1;
% out_li(test_li_indices(neighbouring_indices) + 1) = 1;
% 
% out_li = logical(out_li);

rn = 0.2;
out_li = f_neighbourhood_analysis(sub_pc, test_li, rn);
% out_li = test_li;

toc

f_initFig(1, 'w');
% plot(Xyzti(:, 1), Xyzti(:, 2), '.', 'markersize', 2, 'color', 0.7*[1 1 1])
% plot(Xyzti(li_2, 1), Xyzti(li_2, 2), 'o', 'markersize', 6)
fscatter3_edit_Joona(sub_pc(:, 1), sub_pc(:, 2), sub_pc(:, 3), sub_pc(:, 5), cmap);
plot3(sub_pc(out_li, 1), sub_pc(out_li, 2), sub_pc(out_li, 3), 'ro', 'markersize', 6);

out_text = ['i_th: ', num2str(diff_i_th), '; ', ...
    'grad_z_th: ', num2str(diff_z_th), '; ', 'd_th: ', num2str(d_th)];

disp(out_text);


%%

f_initFig(2, 'w');
li_3 = logical(sub_i_profs == 16 + sub_i_profs(1) - 1);
fscatter3(sub_pc(li_3, 1), sub_pc(li_3, 2), sub_pc(li_3, 3), sub_pc(li_3, 5), cmap)


%%
ticID = tic;

[sub_pc, ~] = f_find_road_raw(Xyzti, ins_prof_pc);

n_skip = 5;

close all;
f_initFig(1, 'k')
fscatter3_edit_Joona(sub_pc(1:n_skip:end, 1), sub_pc(1:n_skip:end, 2), sub_pc(1:n_skip:end, 3), sub_pc(1:n_skip:end, 5), cmap);

t_elapsed_profRetr = toc(ticID);
disp(['Time elapsed: ', num2str(t_elapsed_profRetr), ' seconds.'])

%%
Xyzi = Xyzti(:, [1,2,3,5]);

cumsum_n_pc_profs = cumsum(n_pc_profs);

[sub_pc, sub_i_profs] = f_find_road_raw(Xyzi, ins_prof_pc);

prof_gap = 15;
% mid_prof = 1800;
mid_prof = 2302; %hole
% mid_prof = 3660; %crack
% i_prof = mid_prof;
%%
close all;

for i_prof=mid_prof-prof_gap:mid_prof+prof_gap
    
    Xyzi_prof = sub_pc(logical(sub_i_profs==i_prof), :);
    
    x_prof = Xyzi_prof(:,1);
    y_prof = Xyzi_prof(:,2);
    z_prof = Xyzi_prof(:,3);
    I_prof = Xyzi_prof(:,4);
    
%     close all;
    figure(i_prof-(mid_prof-prof_gap)+1);
    
    i_min = 5;
    i_max = i_min+0;
    for i=i_min:i_max
%         movmean_i = movmean(I_prof, i);
        movmean_z = movmean(z_prof, i);
        movmean_x = movmean(x_prof, i);
        movmean_y = movmean(y_prof, i);
        dist = sqrt( diff(movmean_x).^2 + diff(movmean_y).^2 );
        diff_z = diff(movmean_z);
        
        color_arr = [0 0.5-atan(-(i_max-i_min+1)/8 + (i-i_min)/4)/pi 1];
%         disp([i, color_arr]);
        subplot(221);
        plot(dist, '.', 'color', color_arr);
        title('distance');
        hold on;
        subplot(222);
        plot(movmean_z, '.', 'color', color_arr);
        title('z');
        hold on;
        subplot(223);
%         plot(atan(diff_z./dist)*180/pi, '.', 'color', color_arr);
%         title('angle');
        plot(diff(dist), '.', 'color', color_arr);
        title('difference in distance');
        hold on;
        subplot(224);
        plot(diff_z, '.', 'color', color_arr);
        title('difference in z');
        hold on;
    end
%     hold off;
%     drawnow;
end
% cand = f_roadEdgeCandPts(Xyzi_prof, par_markings);

% disp(cand);

%%
close all;

% f_initFig(1, 'k');
% fscatter3(x_prof, y_prof, z_prof, i_prof, cmap);
%
% plot(z_prof./(mean(z_prof)),'.b')
% hold on;
% plot(i_prof./(mean(i_prof)), 'r.')




%%
% li_test = cand.li_ptsProf_posHeightJump;
% disp(sum(li_test));
% % indices = 1:n_pc;
% % mid_i = indices(logical(sub_i_profs==i_prof));
% mid_i = cumsum_n_pc_profs(i_prof - 1);
% offset = 5e5;
% start_i = mid_i - offset;
% end_i = mid_i + offset;
%
% n_skip = 10;
% ind_fig = ind_fig + 1;
% f_initFig(ind_fig, 'k')
% fscatter3_edit_Joona(Xyzti(start_i:n_skip:end_i, 1), Xyzti(start_i:n_skip:end_i, 2), Xyzti(start_i:n_skip:end_i, 3), Xyzti(start_i:n_skip:end_i, 5), cmap);
% plot3(Xyzi_prof(li_test, 1), Xyzi_prof(li_test, 2), Xyzi_prof(li_test, 3), 'wo', 'linewidth', 1, 'markersize', 6);


