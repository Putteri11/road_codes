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

% min_dist = Inf;
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
% plot(diff(dist_arr), 'b.');
dist_arr = dist_arr(abs(diff(dist_arr))<0.01);
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
rn = dist*1.15; % (metres) 

% query points 

% Q = sub_pc(logical(li), 1:2);
Q = sub_pc([round(sub_n_pc/2), round(sub_n_pc/3)], 1:3);

% point cloud
P = sub_pc;
% P([round(sub_n_pc/2), round(sub_n_pc/3)], :) = [];


%% Find neighbourhood

% create kdtree seach object
ns = createns(P(:, 1:d));

% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q, rn);
ins_neigh_size = size(ins_neigh);
n_neigh = ins_neigh_size(1);

test_arr = zeros(sub_n_pc, 1);
for i_q=1:n_neigh
    search_profs = unique(sub_i_profs(ins_neigh{i_q}));
    assert(length(search_profs)==3);
    q_prof = search_profs(2);
%     search_profs = search_profs([1,3]);
    inds = ins_neigh{i_q}(sub_i_profs(ins_neigh{i_q})~=q_prof);
    test_arr(inds) = inds;
end

test_arr = test_arr(test_arr>0);

%% Plotting (scatter)
n_skip = 1;

ang=0:0.01:2*pi; 
xp=rn*cos(ang);
yp=rn*sin(ang);

f_initFig(1, 'w');
set(gca, 'dataaspectratio', [1 1 1]);
fscatter3_edit_Joona(P(1:n_skip:end, 1), P(1:n_skip:end, 2), P(1:n_skip:end, 3), P(1:n_skip:end, 5), cmap);
for i_q=1:n_neigh
    plot(P(ins_neigh{i_q}, 1), P(ins_neigh{i_q}, 2), 'ro', 'markersize', 6);
    plot(Q(i_q, 1)+xp,Q(i_q, 2)+yp ,'m', 'linewidth', 2);
end
plot(Q(:, 1), Q(:, 2), 'kx', 'markersize', 20, 'linewidth', 2);
plot(P(test_arr, 1), P(test_arr, 2), 'd', 'markersize', 8, 'color', 0.5*[1 1 1], 'linewidth', 2);

