% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Find a local neighourhood for a query point q from a d-dimensional point
% cloud P. P (nxd) is a point cloud matrix, where n is the number of points
% and d  is the dimenstion of the point cloud
%
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

%%

% [li_1, pc_1] = Carve_Several_2D_Objects_edit_Joona(Xyzti(1:10:end, 1:2), 3); % haetaan alueet harvemmasta pilvestä
% li_2 = logical(Carve_Several_2D_Objects(Xyzti(:, 1:2), pc_1)); % haetaan alueissa olevat pisteet koko pilvestä

%% Initialisation

sub_pc = Xyzti(li_2, :);
sub_n_pc = length(sub_pc);
sub_i_profs = ins_prof_pc(li_2);
sub_n_profs = max(sub_i_profs) - sub_i_profs(1) + 1;

n_pc_profs = zeros(sub_n_profs, 1);
first_prof = sub_i_profs(1);
help_var = first_prof;

for ii=1:sub_n_pc
    if sub_i_profs(ii) ~= help_var
        help_var = sub_i_profs(ii);
    end
    n_pc_profs(help_var - first_prof + 1) = n_pc_profs(help_var - first_prof + 1) + 1;
end

tic

n_pc_range = 1:round(mean(n_pc_profs)*2);
sub_sub_pc = sub_pc(n_pc_range, :);
n_pc_profs_cumsum = cumsum(n_pc_profs);
sub_sub_i_profs = sub_i_profs(n_pc_range);

dist_arr = zeros(n_pc_range(end), 1);
prof_range = min(sub_sub_i_profs):max(sub_sub_i_profs)-1;

for prof_i = prof_range
    src_pts = sub_sub_pc(sub_sub_i_profs==prof_i, 1:3);
    dst_pts = sub_sub_pc(sub_sub_i_profs==prof_i+1, 1:3);
    
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
        
        if flag == 0 && exist('mid', 'var')
            min_cand = min([d_1, d_3]);
        end
        
        %         if (min_cand < min_dist)
        %             min_dist = min_cand;
        %         end
        if exist('min_cand', 'var')
            dist_arr(n_pc_profs_cumsum(prof_i-prof_range(1)+1) + i_src) = min_cand;
        end
    end
end

dist_arr = dist_arr(dist_arr>0);
dist_arr = dist_arr(abs(diff(dist_arr))<0.01);
% dist = mean(dist_arr);
% dist_std = std(dist_arr)/2;
% dist_arr = dist_arr(dist_arr<=dist+dist_std | dist_arr>=dist-dist_std);
% dist = mean(dist_arr);
% dist_std = std(dist_arr)/2;
% dist_arr = dist_arr(dist_arr<=dist+dist_std | dist_arr>=dist-dist_std);

% dist_arr = dist_arr(abs(diff(dist_arr))<0.01);
dist = mean(dist_arr);


toc
% disp(dist);

% plot(dist_arr, 'b.');

% f_initFig(1, 'w');
% fscatter3_edit_Joona(sub_pc(n_pc_range, 1), sub_pc(n_pc_range, 2), sub_pc(n_pc_range, 3), sub_pc(n_pc_range, 5), cmap);



%%
% Dimension of the point cloud
d = 3;

% Radius of the local neighbourhood
rn = dist*1.1; % (metres)

% query points

% Q = sub_pc(logical(li), 1:2);
% prof_i = round(mean(unique(sub_i_profs)))+3;
% pc_prof = sub_pc(logical(sub_i_profs==prof_i), :);
% ind1 = round(length(pc_prof(:,1))*0.25);
% ind2 = round(length(pc_prof(:,1))*0.5);
% inds = [ind1, ind2];
inds = 1:1:sub_n_pc;
Q = sub_pc(inds, 1:d);
q_inds = length(Q(:,1));

% point cloud
% P = sub_pc(logical(sub_i_profs==prof_i+1), :);
P = sub_pc;
% P([round(sub_n_pc/2), round(sub_n_pc/3)], :) = [];



%% Find neighbourhood

% create kdtree seach object
% ns = createns(P(:, 1:d));
%
% tic
%
% % Find points in the neighbourhood
% ins_neigh = rangesearch(ns, Q, rn);
%
% toc

tic

diff_z_std_multiplier = 2.5;
diff_z_th_multiplier = 2;
first_prof = sub_i_profs(1);

diff_z_th_across_profs_array = zeros(sub_n_profs, 1);

for prof_i = first_prof:max(sub_i_profs)
    pc_prof = sub_pc(logical(sub_i_profs==prof_i), :);
    
    std_diff_z = std(diff(movmean(pc_prof(:, 3), 5)));
    diff_z_th = std_diff_z * diff_z_std_multiplier;
    
    diff_z_th_across_profs_array(prof_i - first_prof + 1) = ...
        diff_z_th * diff_z_th_multiplier;
end

dist_across_profs = 0.15;
prof_gap = round(dist_across_profs/dist);
neg_jump_inds = zeros(sub_n_pc, prof_gap);
pos_jump_inds = zeros(sub_n_pc, prof_gap);
neg_found = false;
neg_found_prof = 0;
pos_found = false;
found_jump_inds = false;

% jump_inds = zeros(sub_n_pc, 1);
li_neg_jump = false(sub_n_pc, 1);
li_pos_jump = false(sub_n_pc, 1);

% z_diff_arr = zeros(n_neigh, 1);

timestamp_th = 0.0000075;

for i=2:sub_n_profs-1
    prof_i = i-1+first_prof;
    pc_prof = sub_pc(logical(sub_i_profs==prof_i), :);
    l_prof = length(pc_prof(:,1));
    next_pc_prof = sub_pc(logical(sub_i_profs==prof_i+1), :);
    
    for ii = 1:l_prof
        ind = n_pc_profs_cumsum(i-1)+ii;
        
        range_start = max(1, ii-3);
        range_end = min(l_prof, ii+3);
        
        inds_this_prof = range_start:range_end;
        linds_next_prof = abs((next_pc_prof(:,4)-pc_prof(ii,4))-1/f_mirror)<timestamp_th;
        
        z = pc_prof(inds_this_prof, 3);
        z_next = next_pc_prof(linds_next_prof, 3);
        
        z_mean = mean(z);
        z_mean_next = mean(z_next);
        
        z_prof_diff = z_mean_next - z_mean;
        
        col_i = mod(prof_i-1, prof_gap) + 1;
        
        neg_prof_diff = prof_i - neg_found_prof;
        %     pos_prof_diff = q_prof - pos_found_prof;
        %     disp(neg_prof_diff);
        %     ticID = tic;
        if neg_found && neg_prof_diff > prof_gap
            %         disp('hello');
            neg_found = false;
            pos_found = false;
            neg_jump_inds = zeros(sub_n_pc, prof_gap);
            pos_jump_inds = zeros(sub_n_pc, prof_gap);
        end
        %     t_elapsed = toc(ticID);
        %     disp(['Test elapsed: ', num2str(t_elapsed), ' seconds.'])
        
        
        if ~found_jump_inds
            diff_z_th_across_profs = diff_z_th_across_profs_array(prof_i - first_prof + 1);
        else
            diff_z_th_across_profs = diff_z_th_across_profs_array(prof_i - first_prof + 1)*3/4;
        end
        
        if z_prof_diff < -diff_z_th_across_profs
            neg_jump_inds(ind, col_i) = ind;
            neg_found = true;
            neg_found_prof = prof_i;
            found_jump_inds = true;
        elseif (z_prof_diff > diff_z_th_across_profs) && neg_found
            pos_jump_inds(ind, col_i) = ind;
            pos_found = true;
            found_jump_inds = true;
        else
            found_jump_inds = false;
        end
        
        if pos_found && ii==l_prof
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
                        %                     disp(d_test);
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
        
        %     ticID = tic;
        if neg_prof_diff <= prof_gap && ii==l_prof
            neg_jump_inds(:, mod(col_i, prof_gap) + 1) = zeros(sub_n_pc, 1);
        end
        %     t_elapsed = toc(ticID);
        %     disp(['Test elapsed: ', num2str(t_elapsed), ' seconds.'])
        
        %     z_diff_arr(i_q) = z_prof_diff;
        
        %     disp(['difference: ', num2str(z_diff)]);
    end
end

toc

% figure(n_neigh)
% plot(z_diff_arr, 'b.');


%% Plotting (scatter)
n_skip = 1;

% ang=0:0.01:2*pi;
% xp=rn*cos(ang);
% yp=rn*sin(ang);
% test_th = std(z_diff_arr, 'omitnan')*3;
% test_th = diff_z_th*4;

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'w')
set(gca, 'dataaspectratio', [1 1 1]);
fscatter3_edit_Joona(sub_pc(1:n_skip:end, 1), sub_pc(1:n_skip:end, 2), sub_pc(1:n_skip:end, 3), sub_pc(1:n_skip:end, 5), cmap);

% tic
% li = f_find_cracks_and_holes(sub_pc, sub_i_profs);
% plot3(sub_pc(li, 1), sub_pc(li, 2), sub_pc(li, 3), 'ko', 'markersize', 6);
% toc

% tic
% li1 = f_analyze_across_profs(sub_pc, sub_i_profs, dist, ...
%     diff_z_std_multiplier, diff_z_th_multiplier, dist_across_profs);
% plot3(sub_pc(li1, 1), sub_pc(li1, 2), sub_pc(li1, 3), 'ro', 'markersize', 6);
% toc

% ins_neigh = f_find_neighbourhood(sub_pc, sub_pc(li_neg_jump, :), 2*dist);
% li_test1 = f_neighbourhood_analysis(sub_pc, sub_i_profs, li_neg_jump, ins_neigh, 10);
% plot3(sub_pc(li_test1, 1), sub_pc(li_test1, 2), sub_pc(li_test1, 3), 'bo', 'markersize', 6);
plot3(sub_pc(li_neg_jump, 1), sub_pc(li_neg_jump, 2), sub_pc(li_neg_jump, 3), 'bo', 'markersize', 6);
%
% ins_neigh = f_find_neighbourhood(sub_pc, sub_pc(li_pos_jump, :), 2*dist);
% li_test2 = f_neighbourhood_analysis(sub_pc, sub_i_profs, li_pos_jump, ins_neigh, 10);
% plot3(sub_pc(li_test2, 1), sub_pc(li_test2, 2), sub_pc(li_test2, 3), 'ro', 'markersize', 6);
plot3(sub_pc(li_pos_jump, 1), sub_pc(li_pos_jump, 2), sub_pc(li_pos_jump, 3), 'ro', 'markersize', 6);


% for i_q=1:n_neigh
%     plot(P(ins_neigh{i_q}, 1), P(ins_neigh{i_q}, 2), 'ro', 'markersize', 6);
%     plot(Q(i_q, 1)+xp,Q(i_q, 2)+yp ,'m', 'linewidth', 2);
% end
% plot(Q(:, 1), Q(:, 2), 'kx', 'markersize', 20, 'linewidth', 2);
% plot(P(test_arr, 1), P(test_arr, 2), 'd', 'markersize', 8, 'color', 0.5*[1 1 1], 'linewidth', 2);

