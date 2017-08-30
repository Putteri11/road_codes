%%

clear
clc
close all

% addpath('C:\Users\jsa\Documents\MATLAB\tieymparistonMallinnusKoodeja','-end');

%% Parameters etc. 

% plotting stuff
ind_fig = 0; % index of the first figure
cmap = colormap('parula');
close

% Scanner mirror frequency
% f_mirror = 1/0.006665850989521; % (Hz) scanner mirror frequency in Lahti
f_mirror = 1/0.004000438028015; % (Hz) scanner mirror frequency in VT1 (160421_125641) and Nikkila (161124_162516_VUX-1HA.las)
% f_mirror = 2*1/0.004000438028015; % /Hz) scanner mirror frequency in Sipoo

% general parameter values
% par = f_par_roadModelling('VT1'); % parameters for VT1
par = f_par_roadModelling('Nikkila'); % parameters for Nikkilä
par.f_mirror = f_mirror;

% parameters for road surface edge (rse) extraction
% par_rse = f_par_rse('VT1'); % VT1
par_rse = f_par_rse('Nikkila'); % Nikkilä
par_rse.f_mirror = f_mirror;

% parameters for marking extraction
% par_markings = f_par_markings('VT1');
par_markings = f_par_markings('Nikkila');
par_markings.f_mirror = f_mirror;

%% Class labels

% class labels used in the field of the las file for the points in the point cloud
classLabel_unclassified = 0; % unclassified points
classLabel_marking_border_left = 1; % left border marking (painting)
classLabel_marking_border_right = 2; % right border marking (painting)
classLabel_marking_laneSeparator = 3; % markings that separate driving lanes (painting)
classLabel_laneArea = 4; % lane area
classLabel_shouldersVerge1 = 5; % shoulders and verge; side 1 of the road
classLabel_shouldersVerge2 = 6; % shoulders and verge; side 2 of the road
classLabel_roadSurfEdge = 7; % road surface (asphalt) edge points (these are in the same area as shoulders and verge)

% Labels used only temporarily in this script to aid processing
classLabel_marking_long = 8; % long marking;
classLabel_marking_short = 9; % short marking

% Labels for road defects
classLabel_crack_or_hole = 10;


%% Pre-processing

% Remove the following points:
%   Intensity < 800
%   more than one return for the correspondign pulse

% Matlas options:
%   '-keep_single'          Keep only points for which the corresponding pulse has only one return
%   '-drop_intensity_below 800' Remove points whose intensity of the return is below 800

% pre-processing options for matlas
options_las2mat = [' -keep_single -drop_intensity_below ' num2str( par.thre_int )];

%% Load point cloud (las-file)

%   How to deal with several blocks?
%       Overlap needed: Process two of more adjacent blocks at a time?
%   Shoud we load all point cloud blocks here or give file names as iput to
%   the functions?
%       In the latter case, standard file names are needed.
%       If we need to load blocks many times, that may take time.
%       If we load all at once, memory may be a limitation.
%   How to deal with two or more scanners?
%

ticID = tic;


% point cloud file name
% fname_pc = 'E:\Combat_roadModelling_data\RoadDataRoamerR3\VT1\160421_125641_VUX-1-HA_etrs.las'; % VT1
% fname_pc = 'E:\Combat_roadModelling_data\RoadDataRoamerR3\Lahti\160609_150634_VUX-1HA_etrs.las'; % Lahti
% fname_pc = 'C:\Users\jsa\Desktop\04_EXPORT\Nikkila\161124_162516_VUX-1HA.las'; % Nikkila
% fname_pc = 'C:\Users\jsa\Desktop\04_EXPORT\Nikkila\161124_162617_VUX-1HA.las'; % Nikkila
fname_pc = '"C:\Users\jsa\Documents\04_EXPORT\Sipoo170530 - VUX-1HA - 170530_110332_VUX-1HA - originalpoints.las"'; % Sipoo

% home computer
% fname_pc = 'C:\Users\Joona\Documents\GitHub\161124_162516_VUX-1HA.las';

% load and pre-process point cloud
[hdr_pc, pc] = las2mat(['-i ', fname_pc, options_las2mat]);

% elapsed time
t_elapsed_loadPointCloud = toc(ticID);
disp(['Load point cloud ...', fname_pc(end-40:end), ': ', num2str(t_elapsed_loadPointCloud), ' seconds.'])

%% Load the whole trajectory

% Column indices of the trajectory matrix
ci_Traj_x = 16; % x-coordinates (Northing)
ci_Traj_y = 15; % y-coordinates (Easting)
ci_Traj_z = 5; % z-coordinates (Ellipsoidal height)
ci_Traj_t = 2; % time stamps (GPS time)

%
% Traj = load('E:\Combat_roadModelling_data\RoadDataRoamerR3\Lahti\ALS-Lahti_2016_2.mat'); % Lahti
% Traj = Traj.Traj;

% Traj = load('E:\Combat_roadModelling_data\RoadDataRoamerR3\VT1\vt1_21042016.mat'); % VT1
% Traj = Traj.Traj;

Traj = load('C:\Users\jsa\Desktop\04_EXPORT\Nikkila\trajektori_02_FULL\SP_24112016_LIVI.mat'); % Nikkila
Traj = Traj.Traj; 

% home computer
% Traj = load('C:\Users\Joona\Documents\GitHub\trajektori_02_FULL\SP_24112016_LIVI.mat');
% Traj = Traj.Traj; 

%% Organise point cloud into an (nx5) matrix

% Column indices of the input point cloud matrix
ci_Xyzti_x = 1; % x-coordinates
ci_Xyzti_y = 2; % y-coordinates
ci_Xyzti_z = 3; % z-coordinates
ci_Xyzti_t = 4; % time stamps
ci_Xyzti_i = 5; % intensity

Xyzti(:, ci_Xyzti_x) = pc.x;
Xyzti(:, ci_Xyzti_y) = pc.y;
Xyzti(:, ci_Xyzti_z) = pc.z;
Xyzti(:, ci_Xyzti_t) = pc.gps_time;
Xyzti(:, ci_Xyzti_i) = pc.intensity;

%% Sort points based on time stamps

if ~issorted( Xyzti(:, ci_Xyzti_t) )
    [~, ins_sort] = sort( Xyzti(:, ci_Xyzti_t) );
    Xyzti = Xyzti( ins_sort, : );
    pc.x = pc.x(ins_sort);
    pc.y = pc.y(ins_sort);
    pc.z = pc.z(ins_sort);
    pc.intensity = pc.intensity(ins_sort);
    pc.gps_time = pc.gps_time(ins_sort);
    
    pc.return_number = pc.return_number(ins_sort);
    pc.number_of_returns = pc.number_of_returns(ins_sort);
    pc.scan_direction_flag = pc.scan_direction_flag(ins_sort);
    pc.edge_of_flight_line = pc.edge_of_flight_line(ins_sort);
    pc.classification = pc.classification(ins_sort);
    pc.scan_angle_rank = pc.scan_angle_rank(ins_sort);
    pc.user_data = pc.user_data(ins_sort);
    pc.point_source_ID = pc.point_source_ID(ins_sort);
end

if ~issorted( Traj(:, ci_Traj_t) )
    [~, ins_sort] = sort( Traj(:, ci_Traj_t) );
    Traj = Traj( ins_sort, : );
end

%% Take only a subset of points for testing

% Take n_subset first points
% start = 1;
% n_subset = size( Xyzti, 1 ); % take all
start = size( Xyzti, 1 ) / 2;
n_subset = 1e+7;
subset_range = start:start+n_subset-1;

Xyzti = Xyzti(subset_range, :);

% original
pc.x = pc.x(subset_range);
pc.y = pc.y(subset_range);
pc.z = pc.z(subset_range);
pc.intensity = pc.intensity(subset_range);
pc.gps_time = pc.gps_time(subset_range);

pc.return_number = pc.return_number(subset_range);
pc.number_of_returns = pc.number_of_returns(subset_range);
pc.scan_direction_flag = pc.scan_direction_flag(subset_range);
pc.edge_of_flight_line = pc.edge_of_flight_line(subset_range);
pc.classification = pc.classification(subset_range);
pc.scan_angle_rank = pc.scan_angle_rank(subset_range);
pc.user_data = pc.user_data(subset_range);
pc.point_source_ID = pc.point_source_ID(subset_range);

% number of points in the point cloud
n_pc = length(pc.x);


%% Transform trajectory time stamps to the same "time zone" with the point cloud

Traj(:, ci_Traj_t) = Traj(:, ci_Traj_t) - 17; % 17 seconds difference

%% Select subset of the trajectory which overlaps with the point cloud

% This is important if the same area is driven two times. In that case the
% correct drive has to be selected here

% plot point cloud and whole trajectory 
ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'w')
plot3(Xyzti(:, 1), Xyzti(:, 2), Xyzti(:, 3), '.', 'markersize', 2)
plot3(Traj(:,ci_Traj_x), Traj(:,ci_Traj_y), Traj(:,ci_Traj_z))

% find overlapping part of the traj automatically
t_pc_first = Xyzti(1, 4); % time stamp of the first point in the point cloud
t_pc_last = Xyzti(end, 4); % time stamp of the last point in the point cloud
t_extra = 2; % (seconds) take some extra trajectory on both sides of the point cloud ??lisää parametriksi inputtiin
li_Traj_overlapPC = Traj(:, ci_Traj_t) >= t_pc_first - t_extra & Traj(:, ci_Traj_t) <= t_pc_last + t_extra; % li for traj points; true indicates that the point overlaps point cloud
Traj = Traj(li_Traj_overlapPC, :); % select only overlapping part of the trajectory

% plot the selected part of the trajectory; this should overlap with the
% point cloud
plot3(Traj(:,ci_Traj_x), Traj(:,ci_Traj_y), Traj(:,ci_Traj_z), 'linewidth', 2)
legend('Point cloud', 'Whole trajectory', 'Overlapping part of the trajectory') 

% check time stamps 
xlim_min = min(Traj(:, ci_Traj_t)) - 1;
xlim_max = max(Traj(:, ci_Traj_t)) + 1;
figure
subplot(2,1,1)
hold on
title('point cloud time stamps')
xlim([xlim_min, xlim_max]) % make sure both subplots have same x lim
histogram( pc.gps_time )
subplot(2,1,2)
hold on
title('overlapping traj time stamps')
histogram( Traj(:, ci_Traj_t) )
xlim([xlim_min, xlim_max]) % make sure both subplots have same x lim

%% Retrieve profiles
% with all points: Profile retrieval: 928.1594 seconds. (15 min 30 s)
ticID = tic;
[ins_prof_pc, ins_pc_profStart, ins_pc_profStart_reset, ins_prof_reset] = f_retrProfiles(Xyzti(:, [ci_Xyzti_x, ci_Xyzti_y, ci_Xyzti_z, ci_Xyzti_t]), Traj(:, [ci_Traj_x, ci_Traj_y, ci_Traj_z, ci_Traj_t]), par.f_mirror);
t_elapsed_profRetr = toc(ticID);
disp(['Profile retrieval: ', num2str(t_elapsed_profRetr), ' seconds.'])

% number of profiles
n_profs = max(ins_prof_pc);

%% Testing f_find_cracks_and_holes (1/2)

tic
[li_1, pc_1] = Carve_Several_2D_Objects_edit_Joona(Xyzti(1:10:end, 1:2), 3); % haetaan alueet harvemmasta pilvestä
li_2 = logical(Carve_Several_2D_Objects(Xyzti(:, 1:2), pc_1)); % haetaan alueissa olevat pisteet koko pilvestä
toc

%% (2/2)

sub_pc = Xyzti(li_2, :);
sub_i_profs = ins_prof_pc(li_2);

ticID = tic;
li = f_find_cracks_and_holes(sub_pc, sub_i_profs, f_mirror);
t_elapsed_profRetr = toc(ticID);
disp(['Defect retrieval: ', num2str(t_elapsed_profRetr), ' seconds.'])

n_skip = 1;
ind_fig = ind_fig + 1;
user_input = input('All points? (y/[n]) ', 's');
if user_input == 'y'
    f_initFig(ind_fig, 'k')
    fscatter3_edit_Joona(Xyzti(1:n_skip:end, 1), Xyzti(1:n_skip:end, 2), Xyzti(1:n_skip:end, 3), Xyzti(1:n_skip:end, 5), cmap);
    plot3(sub_pc(logical(li), 1), sub_pc(logical(li), 2), sub_pc(logical(li), 3), 'wo', 'linewidth', 1, 'markersize', 6);
else
    f_initFig(ind_fig, 'w')
    fscatter3_edit_Joona(sub_pc(1:n_skip:end, 1), sub_pc(1:n_skip:end, 2), sub_pc(1:n_skip:end, 3), sub_pc(1:n_skip:end, 5), cmap);
    plot3(sub_pc(logical(li), 1), sub_pc(logical(li), 2), sub_pc(logical(li), 3), 'ro', 'linewidth', 1, 'markersize', 6);
end
xlabel('x');
ylabel('y');
zlabel('z');

%% Testing f_find_cracks_and_holes on the whole road (rough)

[sub_pc, sub_i_profs] = f_find_road_raw(Xyzti, ins_prof_pc);

ticID = tic;

li = f_find_cracks_and_holes(sub_pc, sub_i_profs, f_mirror);
t_elapsed_profRetr = toc(ticID);
disp(['Defect retrieval: ', num2str(t_elapsed_profRetr), ' seconds.'])

n_skip = 10;
ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'k')
fscatter3_edit_Joona(Xyzti(1:n_skip:end, 1), Xyzti(1:n_skip:end, 2), Xyzti(1:n_skip:end, 3), Xyzti(1:n_skip:end, 5), cmap);
plot3(sub_pc(logical(li), 1), sub_pc(logical(li), 2), sub_pc(logical(li), 3), 'wo', 'linewidth', 1, 'markersize', 6);

%% Plot point cloud and profile stuff

% plot startint point of each profile; this way you can test if the profile
% retrieval works ok

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'k')
n_skip_pc_plot = 10; % plot only every n_skip_pc_plot points
% plot3(Xyzti(1:n_skip_pc_plot:end, 1), Xyzti(1:n_skip_pc_plot:end, 2), Xyzti(1:n_skip_pc_plot:end, 3), '.', 'markersize', 2, 'color', 0.75*[1 1 1])
fscatter3_edit_Joona(Xyzti(1:n_skip_pc_plot:end, 1), Xyzti(1:n_skip_pc_plot:end, 2), Xyzti(1:n_skip_pc_plot:end, 3), Xyzti(1:n_skip_pc_plot:end, 5), cmap);
plot3(Xyzti(ins_pc_profStart, 1), Xyzti(ins_pc_profStart, 2), Xyzti(ins_pc_profStart, 3), 'wx', 'linewidth', 2, 'markersize', 20)


%% Write classified point cloud to ta las-file

% classification
% li_pc_roadDefect = li;
% pc.classification(li_pc_roadDefect) = classLabel_crack_or_hole;

% % write las file
% mat2las(pc, '-fgi_scale 0.001 0.001 0.001 -o C:\Users\jsa\Desktop\04_EXPORT\Nikkila\161124_162516_VUX-1HA_cracks_and_holes1.las');

