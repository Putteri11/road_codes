%%

clear
clc
close all

%% Parameters etc. 

% plotting stuff
ind_fig = 0; % index of the first figure
cmap = colormap('parula');
close

% Scanner mirror frequency
% f_mirror = 1/0.006665850989521; % (Hz) scanner mirror frequency in Lahti
f_mirror = 1/0.004000438028015; % (Hz) scanner mirror frequency in VT1 (160421_125641) and Nikkila (161124_162516_VUX-1HA.las)

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
fname_pc = 'C:\Users\jsa\Desktop\04_EXPORT\Nikkila\161124_162516_VUX-1HA.las'; % Nikkila
% fname_pc = 'C:\Users\jsa\Desktop\04_EXPORT\Nikkila\161124_162617_VUX-1HA.las'; % Nikkila

% load and pre-process point cloud
[hdr_pc, pc] = las2mat(['-i ', fname_pc, options_las2mat]);

% elapsed time
t_elapsed_loadPointCloud = toc(ticID);
disp(['Load point cloud ...', fname_pc(end-40:end), ': ', num2str(t_elapsed_loadPointCloud), ' seconds.'])


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

%% Take only a subset of points for testing

% Take n_first_subset first points
% n_first_subset = size( Xyzti, 1 ); % take all
n_first_subset = 1e+6;

Xyzti = Xyzti(1:n_first_subset, :);

pc.x = pc.x(1:n_first_subset);
pc.y = pc.y(1:n_first_subset);
pc.z = pc.z(1:n_first_subset);
pc.intensity = pc.intensity(1:n_first_subset);
pc.gps_time = pc.gps_time(1:n_first_subset);

pc.return_number = pc.return_number(1:n_first_subset);
pc.number_of_returns = pc.number_of_returns(1:n_first_subset);
pc.scan_direction_flag = pc.scan_direction_flag(1:n_first_subset);
pc.edge_of_flight_line = pc.edge_of_flight_line(1:n_first_subset);
pc.classification = pc.classification(1:n_first_subset);
pc.scan_angle_rank = pc.scan_angle_rank(1:n_first_subset);
pc.user_data = pc.user_data(1:n_first_subset);
pc.point_source_ID = pc.point_source_ID(1:n_first_subset);

% number of points in the point cloud
n_pc = length(pc.x);

%% Load trajectory

% Column indices of the trajectory matrix
ci_Traj_x = 16; % x-coordinates (Northing)
ci_Traj_y = 15; % y-coordinates (Easting)
ci_Traj_z = 5; % z-coordinates (Ellipsoidal height)
ci_Traj_t = 2; % time stamps (GPS time)

%
% Traj = load('E:\Combat_roadModelling_data\RoadDataRoamerR3\Lahti\ALS-Lahti_2016_2.mat'); % Lahti
% Traj = Traj.Traj;
%


% Traj = load('E:\Combat_roadModelling_data\RoadDataRoamerR3\VT1\vt1_21042016.mat'); % VT1
% Traj = Traj.Traj;


Traj = load('C:\Users\jsa\Desktop\04_EXPORT\Nikkila\trajektori_02_FULL\SP_24112016_LIVI.mat'); % Nikkila
Traj = Traj.Traj(1:5e+5, :); % take the correct driving direction

%% Transform trajectory time stamps to the same "time zone" with the point cloud

Traj(:, ci_Traj_t) = Traj(:, ci_Traj_t) - 17; % 17 seconds difference

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

%% Retrieve profiles

ticID = tic;
[ins_prof_pc, ins_pc_profStart, ins_pc_profStart_reset, ins_prof_reset] = f_retrProfiles(Xyzti(:, [ci_Xyzti_x, ci_Xyzti_y, ci_Xyzti_z, ci_Xyzti_t]), Traj(:, [ci_Traj_x, ci_Traj_y, ci_Traj_z, ci_Traj_t]), par.f_mirror);
t_elapsed_profRetr = toc(ticID);
disp(['Profile retrieval: ', num2str(t_elapsed_profRetr), ' seconds.'])

% number of profiles
n_profs = max(ins_prof_pc);

%% Plot point cloud and profile stuff

% plot startint point of each profile; this way you can test if the profile
% retrieval works ok

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'k')
n_skip_pc_plot = 5; % plot only every n_skip_pc_plot points
% plot3(Xyzt(1:n_skip_pc_plot:end, 1), Xyzt(1:n_skip_pc_plot:end, 2), Xyzt(1:n_skip_pc_plot:end, 3), '.', 'markersize', 2, 'color', 0.75*[1 1 1])
fscatter3(Xyzti(1:n_skip_pc_plot:end, 1), Xyzti(1:n_skip_pc_plot:end, 2), Xyzti(1:n_skip_pc_plot:end, 3), pc.intensity(1:n_skip_pc_plot:end), cmap);
plot3(Xyzti(ins_pc_profStart, 1), Xyzti(ins_pc_profStart, 2), Xyzti(ins_pc_profStart, 3), 'wx', 'linewidth', 2, 'markersize', 20)


%% Write classified point cloud to ta las-file

% classification
% pc.classification(li_pc_roadSurfEdge) = classLabel_roadSurfEdge;

% % write las file
% mat2las(pc, '-fgi_scale 0.001 0.001 0.001 -o E:\Combat_roadModelling_data\RoadDataRoamerR3\VT1\160421_125641_VUX-1-HA_etrs_edges_6.las');

