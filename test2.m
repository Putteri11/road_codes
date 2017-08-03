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

% Dimension of the point cloud
d = 2;

% Radius of the local neighbourhood
rn = 0.06; % (metres) 

% query points 

% Q = sub_pc(logical(li), 1:2);
Q = sub_pc([round(sub_n_pc/2), round(sub_n_pc/3)], 1:2);

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

% test_arr = zeros(sub_n_pc, 1);
for ii=1:n_neigh
    search_profs = unique(sub_i_profs(ins_neigh{ii}));
    assert(length(search_profs)==3);
%     q_prof = search_profs(2);
    search_profs = search_profs([1,3]);
    inds1 = ins_neigh{i_q}(sub_i_profs(ins_neigh{i_q})==search_profs(1));
    inds2 = ins_neigh{i_q}(sub_i_profs(ins_neigh{i_q})==search_profs(2));
%     test_arr(inds) = inds;
    % min distance...
    dist1 = zeros(length(inds1), 1);
    dist2 = zeros(length(inds2), 1);
    for i_ind = 1:length(inds1)
        % vertcat
        % diff 
        % .^2
        % sum
        % sqrt
    end
    for i_ind = 1:length(inds2)
        
    end
    min_dist = mean(min(dist1), min(dist2));
end
% test_arr = test_arr(test_arr>0);

%% Plotting (scatter)
n_skip = 1;

ang=0:0.01:2*pi; 
xp=rn*cos(ang);
yp=rn*sin(ang);

f_initFig(1, 'w');
set(gca, 'dataaspectratio', [1 1 1]);
fscatter3_edit_Joona(P(1:n_skip:end, 1), P(1:n_skip:end, 2), P(1:n_skip:end, 3), P(1:n_skip:end, 5), cmap);
for ii=1:n_neigh
    plot(P(ins_neigh{ii}, 1), P(ins_neigh{ii}, 2), 'ro', 'markersize', 6);
    plot(Q(ii, 1)+xp,Q(ii, 2)+yp ,'m', 'linewidth', 2);
end
plot(Q(:, 1), Q(:, 2), 'kx', 'markersize', 20, 'linewidth', 2);
% plot(P(test_arr, 1), P(test_arr, 2), 'd', 'markersize', 8, 'color', 0.5*[1 1 1], 'linewidth', 2);

