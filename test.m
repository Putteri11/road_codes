
ticID = tic;
n_pc_profs = zeros(n_profs, 1);
first_prof = ins_prof_pc(1);
help_var = first_prof;

for ii=1:n_pc
    if ins_prof_pc(ii) ~= help_var
        help_var = ins_prof_pc(ii);
    end
    n_pc_profs(help_var - first_prof + 1) = n_pc_profs(help_var - first_prof + 1) + 1;
end
% n_pc_profs = n_pc_profs(n_pc_profs>0);

% plot(n_pc_profs, 'b.')

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
n_pc_profs = zeros(sub_n_profs, 1);
first_prof = sub_i_profs(1);
help_var = first_prof;

for ii=1:sub_n_pc
    if sub_i_profs(ii)==help_var
        n_pc_profs(help_var - first_prof + 1) = n_pc_profs(help_var - first_prof + 1) + 1;
    else
        help_var = sub_i_profs(ii);
        n_pc_profs(help_var - first_prof + 1) = n_pc_profs(help_var - first_prof + 1) + 1;
    end
end

n_pc_range = 1:round(mean(n_pc_profs)*2);
sub_sub_pc = sub_pc(n_pc_range, :);
n_pc_profs_cumsum = cumsum(n_pc_profs);
sub_sub_i_profs = sub_i_profs(n_pc_range);

dist_arr = zeros(n_pc_range(end), 1);
prof_range = min(sub_sub_i_profs):max(sub_sub_i_profs)-1;

for i_prof = prof_range
    src_pts = sub_sub_pc(sub_sub_i_profs==i_prof, 1:3);
    dst_pts = sub_sub_pc(sub_sub_i_profs==i_prof+1, 1:3);
    
    for i_src = 1:10:length(src_pts(:,1))
        % binary search
        src = src_pts(i_src, :);
        left = 1;
        right = length(dst_pts(:,1));
        flag = 0;
        
        while left <= right - 2
            mid = ceil((left + right) / 2);
            d_1 = sqrt(sum(diff(vertcat(src, dst_pts(mid-1, :))).^2));
            d_2 = sqrt(sum(diff(vertcat(src, dst_pts(mid, :))).^2));
            d_3 = sqrt(sum(diff(vertcat(src, dst_pts(mid+1, :))).^2));
            
            if (d_2 <= d_1) && (d_2 <= d_3)
                min_cand = d_2;
                flag = 1;
                break;
            elseif (d_2 > d_3) || (d_1 > d_2)
                left = mid + 1;
            else
                right = mid - 1;
            end
        end
        
        if flag == 0
            min_cand = min([d_1, d_3]);
        end
        
        %         if (min_cand < min_dist)
        %             min_dist = min_cand;
        %         end
        dist_arr(n_pc_profs_cumsum(i_prof-prof_range(1)+1) + i_src) = min_cand;
    end
end

dist_arr = dist_arr(dist_arr>0);
dist_arr = dist_arr(abs(diff(dist_arr))<0.01);
dist = mean(dist_arr);


% monte carlo
% i_th = 1600 + 50*randn;
% grad_z_th = 0.0004 + 0.0001*randn;
% d_th = 3.0 + 0.2*randn;
% percentage = 0.4 + 0.05*randn;


% fixed values
i_th = 2100;
diff_i_th = 130;
diff_z_th = 0.0023; % 0.0023...0.0028
% diff_z_th2 = 0.0018;
% window = 20;
% d_th = 0.028;

li_cand = false(sub_n_pc, 1);
helper = cumsum(n_pc_profs);

for i=2:sub_n_profs-1
    prof_i = i-1+first_prof;
    pc_prof = sub_pc(logical(prof_i==sub_i_profs), :);
    l_prof = length(pc_prof(:, 1));
    
    prof_range = 2:l_prof-1;
    last_ind = 1;
    
    while last_ind < prof_range(end)
        
        [jump_inds, found_jump_inds, last_ind] = f_analyze_prof(pc_prof, ...
            helper(i - 1), diff_z_th, prof_range);

        if found_jump_inds
            next_pc_prof = sub_pc(logical(prof_i+1==sub_i_profs), :);
            Q = pc_prof(jump_inds(1:end-1:end) - helper(i - 1), 1:3);
            rn = dist*1.5;
            ins_neigh_next_prof = f_find_neighbourhood(next_pc_prof, Q, rn);
            ins_next_prof_range = min(ins_neigh_next_prof{1}):max(ins_neigh_next_prof{2})-1;
            
            diff_z_th2 = diff_z_th*3/4;
            
            [jump_inds_next_prof, found_jump_inds_next_prof, ~] = ...
                f_analyze_prof(next_pc_prof, helper(i), diff_z_th2, ins_next_prof_range);
            
            if found_jump_inds_next_prof
                li_cand(jump_inds) = true;
                li_cand(jump_inds_next_prof) = true;
            end
        end
        
        prof_range = last_ind:prof_range(end);
        
    end
    
  
end


% test_li_indices = find(li_cand == 1, sub_n_pc);
% neighbouring_indices = find(diff(test_li_indices) == 1, length(test_li_indices));
%
% out_li = zeros(sub_n_pc, 1);
% out_li(test_li_indices(neighbouring_indices)) = 1;
% out_li(test_li_indices(neighbouring_indices) + 1) = 1;
%
% out_li = logical(out_li);

rn = dist*1.15;
n_points_th = 1;
ins_neigh = f_find_neighbourhood(sub_pc, sub_pc(li_cand, 1:3), rn);
out_li = f_neighbourhood_analysis(sub_pc, sub_i_profs, li_cand, ins_neigh, n_points_th);

% out_li = li_cand;

rn = dist*4;
n_points_th = 20;
ins_neigh = f_find_neighbourhood(sub_pc, sub_pc(out_li, 1:3), rn);
out_li = f_neighbourhood_analysis(sub_pc, sub_i_profs, out_li, ins_neigh, n_points_th);


toc

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'w');
% plot(Xyzti(:, 1), Xyzti(:, 2), '.', 'markersize', 2, 'color', 0.7*[1 1 1])
% plot(Xyzti(li_2, 1), Xyzti(li_2, 2), 'o', 'markersize', 6)
fscatter3_edit_Joona(sub_pc(:, 1), sub_pc(:, 2), sub_pc(:, 3), sub_pc(:, 5), cmap);
plot3(sub_pc(out_li, 1), sub_pc(out_li, 2), sub_pc(out_li, 3), 'ro', 'markersize', 6);

% out_text = ['i_th: ', num2str(diff_i_th), '; ', ...
%     'grad_z_th: ', num2str(diff_z_th), '; ', 'd_th: ', num2str(d_th)];
%
% disp(out_text);


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
% Xyzi = Xyzti(:, [1,2,3,5]);

% cumsum_n_pc_profs = cumsum(n_pc_profs);

% [sub_pc, sub_i_profs] = f_find_road_raw(Xyzi, ins_prof_pc);

prof_gap = 15;
% mid_prof = 1800;
mid_prof = sub_i_profs(round(end/2));
% mid_prof = 2302; %hole
% mid_prof = 3660; %crack
% i_prof = mid_prof;
%%
close all;

tic

for i_prof=mid_prof-prof_gap:mid_prof+prof_gap
    
    Xyzi_prof = sub_pc(logical(sub_i_profs==i_prof), :);
    
    x_prof = Xyzi_prof(:,1);
    y_prof = Xyzi_prof(:,2);
    z_prof = Xyzi_prof(:,3);
    I_prof = Xyzi_prof(:,4);
    
    %     close all;
    figure(i_prof-(mid_prof-prof_gap)+1);
    
    i_min = 5;
    gap = 5;
    i_max = i_min+gap;
    for i=i_min:i_max
        %         movmean_i = movmean(I_prof, i);
        movmean_z = movmean(z_prof, i);
        movmean_x = movmean(x_prof, i);
        movmean_y = movmean(y_prof, i);
        dist = sqrt( diff(movmean_x).^2 + diff(movmean_y).^2 );
        diff_z = diff(movmean_z);
        
        k = -1/(i_max - i_min);
        c = i_max/(i_max - i_min);
        color_arr = [0 k*i+c 1];
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

toc
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


