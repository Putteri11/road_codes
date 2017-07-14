% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% 
% Find a local neighourhood for a query point q from a d-dimensional point
% cloud P. P (nxd) is a point cloud matrix, where n is the number of points
% and d  is the dimenstion of the point cloud
% 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%% Initialisation

clear

% Dimension of the point cloud
d = 2;

% number of points in the point cloud
n = 1000;

% Radius of the local neighbourhood
rn = 0.2; % (metres) 

% point cloud
P = rand(n, d);

% query points 
Q = [0 0.75; 0.5 0.6; 0.9 0];


%% Find neighbourhood

% create kdtree seach object
ns = createns(P(:, 1:d));
   
% Find points in the neighbourhood
ins_neigh = rangesearch(ns, Q, rn);

%% Plotting (2D)

figure
hold on
set(gca, 'dataaspectratio', [1 1 1])
plot(P(:, 1), P(:, 2), '.')
plot(P(ins_neigh{1}, 1), P(ins_neigh{1}, 2), 'o', 'markersize', 6)
plot(P(ins_neigh{2}, 1), P(ins_neigh{2}, 2), 'o', 'markersize', 6)
plot(P(ins_neigh{3}, 1), P(ins_neigh{3}, 2), 'o', 'markersize', 6)
plot(Q(:, 1), Q(:, 2), 'x', 'markersize', 20, 'linewidth', 2)

%% Plotting (3D)

% figure
% hold on
% set(gca, 'dataaspectratio', [1 1 1])
% plot3(P(:, 1), P(:, 2), P(:, 3), '.')
% plot3(P(Ins_neigh{1}, 1), P(Ins_neigh{1}, 2), P(Ins_neigh{1}, 3), 'o')
% plot3(q(1), q(2), q(3), 'x', 'markersize', 20, 'linewidth', 2)
