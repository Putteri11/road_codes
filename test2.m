% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% 
% Find a local neighourhood for a query point q from a d-dimensional point
% cloud P. P (nxd) is a point cloud matrix, where n is the number of points
% and d  is the dimenstion of the point cloud
% 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%%

[li_1, pc_1] = Carve_Several_2D_Objects_edit_Joona(Xyzti(1:10:end, 1:2), 3); % haetaan alueet harvemmasta pilvestä
li_2 = logical(Carve_Several_2D_Objects(Xyzti(:, 1:2), pc_1)); % haetaan alueissa olevat pisteet koko pilvestä

%% Initialisation

sub_pc = Xyzti(li_2, :);
sub_n_pc = length(sub_pc);
sub_i_profs = ins_prof_pc(li_2);
sub_n_profs = max(sub_i_profs) - sub_i_profs(1) + 1;

% Dimension of the point cloud
d = 2;

indices = 1:sub_n_pc;
sub_pc_w_i = [sub_pc, (indices)'];

% Radius of the local neighbourhood
rn = 0.2; % (metres) 

% point cloud
P = sub_pc_w_i;

% query points 

Q = sub_pc(logical(li), 1:2);


%% Find neighbourhood

% create kdtree seach object
ns = createns(P(:, 1:d));
   
% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q, rn);
ins_neigh_size = size(ins_neigh);
n_neigh = ins_neigh_size(1);

%% Plotting (2D)
n_skip = 5;

f_initFig(1, 'w')
set(gca, 'dataaspectratio', [1 1 1])
plot(P(1:n_skip:end, 1), P(1:n_skip:end, 2), '.b', 'markersize', 2)
for ii=1:n_neigh
    plot(P(ins_neigh{ii}, 1), P(ins_neigh{ii}, 2), 'ro', 'markersize', 6)
end
plot(Q(:, 1), Q(:, 2), 'kx', 'markersize', 20, 'linewidth', 2)

%% Plotting (3D)

n_skip = 5;

f_initFig(1, 'w')
set(gca, 'dataaspectratio', [1 1 1])
plot3(P(1:n_skip:end, 1), P(1:n_skip:end, 2), P(1:n_skip:end, 3), 'b.', 'markersize', 2)
for ii=1:n_neigh
    plot3(P(ins_neigh{ii}, 1), P(ins_neigh{ii}, 2), P(ins_neigh{ii}, 3), 'ro', 'markersize', 6)
    Q(ii, 3) = mean(P(ins_neigh{ii}, 3));
end
plot3(Q(:, 1), Q(:, 2), Q(:, 3), 'kx', 'markersize', 20, 'linewidth', 2)
