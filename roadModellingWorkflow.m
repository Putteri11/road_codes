% % % % % % % % % % % % % % % % % % % % % 
% 
% Notes:
%   -   6.7.2017: A bug fix (changed order of sections such that time
%       stamps are sorted (and points based on them) before a subset of
%       data is selected. (sama korjattu myös koodiin
%       mls_preProcessing_1.m)
%   -   7.7.2017: The part of the trajectory, which overlaps with the
%       trajectory is retrieved automatically based on time stamps
% 
% 
% 
% % % % % % % % % % % % % % % % % % % % % 


%% Initialisation

clear
clc 
close all

addpath('C:\Users\jsa\Documents\MATLAB\tieymparistonMallinnusKoodeja','-end');

%% Parameters etc.

% plotting stuff
ind_fig = 0; % index of the first figure 
cmap = colormap('parula');
close

% ??ao. tapa ei välttämättä toimi järkevästi, jos saman kokonaisuuden
% sisällä (esim. rse) joudutaan kutsumaan samaa funktiota, mutta eri
% parametriarviolla JA jos funktiolle syötetään koko parametri structi.
% Tällöin parametriarvo pitää laittaa ko. funktion inputtiin erikseen ja
% laittaa structiin kaksi eri kenttää ko. parametrille.

% Scanner mirror frequency ??keksitkö järkevän tavan ympätä tämän
% parametristrukteihin s.e. ei tarvii toistaa samaa riviä moneen kertaan
% (vai haittaako). Periaatteessa voi olla että tämä estimoidaan datasta.
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

%% Load whole trajectory

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
Traj = Traj.Traj; 

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

ticID = tic;
[ins_prof_pc, ins_pc_profStart, ins_pc_profStart_reset, ins_prof_reset] = f_retrProfiles(Xyzti(:, [ci_Xyzti_x, ci_Xyzti_y, ci_Xyzti_z, ci_Xyzti_t]), Traj(:, [ci_Traj_x, ci_Traj_y, ci_Traj_z, ci_Traj_t]), par.f_mirror);
t_elapsed_profRetr = toc(ticID);
disp(['Profile retrieval: ', num2str(t_elapsed_profRetr), ' seconds.'])

% number of profiles
n_profs = max(ins_prof_pc);

%% Plot profile stuff

% plot startint point of each profile; this way you can test if the profile
% retrieval works ok 

% ind_fig = ind_fig + 1;
% f_initFig(ind_fig, 'w')
% plot3(Xyzti(1:32:end, 1), Xyzti(1:32:end, 2), Xyzti(1:32:end, 3), '.', 'markersize', 2, 'color', 0.75*[1 1 1])
% plot3(Xyzti(ins_pc_profStart, 1), Xyzti(ins_pc_profStart, 2), Xyzti(ins_pc_profStart, 3), 'x', 'linewidth', 2, 'markersize', 20)

%% Find approximate road centre line

% [rcl, stuff] = f_roadCentreLineRough(Xyzti, Traj(:, [ci_Traj_x, ci_Traj_y, ci_Traj_z, ci_Traj_t]), ins_Xyzti_prof, par);


%% Extract road marking candidates

ticID = tic;

% ??Remove points that correspond to pulses with two or more returns?



% Allocate memory for road marking candidates. Each row equals
% [ind_point_posJump, ind_point_nextNegJump],
% where ind_point_posJump is the index of the point (in pc.x etc.) where
% the intensity rises, ind_point_nextNegJump is the
% index of the point where the intensity drops
RoadMarkingCand_1 = nan( round( size(Xyzti, 1)/100 ) , 2 );

% version 2: marking candidates in a cell array, each row corresponds to
% one profile. First column contains the positive jumps and second the
% negative jumps.
roadMarkingCand_2 = cell( n_profs , 2 ); % row indices correspond to the profile indices in ins_prof_pc

% initialise index of the last added road marking.
ind_lastMarking = 0;

% Extract candidate points from one profile (this one will come inside some
% other function)
for ind_prof = 1 : 1 : n_profs % index of the profile
    
    % ind_prof
    
    li_Xyzti_prof = ins_prof_pc == ind_prof; % logical indices for Xyzti; true indicates that the point belongs to the current profile
    ins_Xyzti_prof = find( li_Xyzti_prof ); % linear indices of the profile points
    n_pts_prof = sum( li_Xyzti_prof ); % number of points on the profile
    
    % ??saat luultavasti nopeutettua ao. riviä, jos otat suoraan
    % Xyzi-matriisista (siirrä se alkuun ja ite asiassa tee Xyzit-matriisi)
    
    Xyzi_prof = [ pc.x(li_Xyzti_prof), pc.y(li_Xyzti_prof), pc.z(li_Xyzti_prof), pc.intensity(li_Xyzti_prof) ]; % points on the profile (xyz) These are sorted based on time stamps
    
    % Plot adjacent profiles
    % f_plotAdjProfs(ind_prof, 50, 202, pc, ins_prof_pc);
    
    % extract road edge etc. candidates
    cand = f_roadEdgeCandPts(Xyzi_prof, par_markings); % input is one profile
    
    % road marking candidates
    n_markingCand_profile = size( cand.RoadMarkings, 1 ); % number of marking candidates on the current profile
    if n_markingCand_profile > 0
        RoadMarkingCand_1(ind_lastMarking + 1 : ind_lastMarking + n_markingCand_profile, 1) = ins_Xyzti_prof( cand.RoadMarkings(:, 1) ); % positive intensity jumps
        RoadMarkingCand_1(ind_lastMarking + 1 : ind_lastMarking + n_markingCand_profile, 2) = ins_Xyzti_prof( cand.RoadMarkings(:, 2) ); % negative intensity jumps
        roadMarkingCand_2{ind_prof, 1} = ins_Xyzti_prof( cand.RoadMarkings(:, 1) ); % positive intensity jumps
        roadMarkingCand_2{ind_prof, 2} = ins_Xyzti_prof( cand.RoadMarkings(:, 2) ); % negative intensity jumps
        ind_lastMarking = ind_lastMarking + n_markingCand_profile; % update index of the last added marking
    end
    
end

RoadMarkingCand_1 = RoadMarkingCand_1(1:ind_lastMarking, :);

% elapsed time 
t_elapsed_markingCands = toc(ticID);
disp(['Marking candidates: ', num2str(t_elapsed_markingCands), ' seconds.'])

%% Plot candidates

% ind_fig = ind_fig + 1;
% f_initFig(ind_fig, 'k')
% plot3(Xyzti(RoadMarkingCand_1(:, 1), 1), Xyzti(RoadMarkingCand_1(:, 1), 2), Xyzti(RoadMarkingCand_1(:, 1), 3), 'gx', 'linewidth', 2, 'markersize', 20)
% plot3(Xyzti(RoadMarkingCand_1(:, 2), 1), Xyzti(RoadMarkingCand_1(:, 2), 2), Xyzti(RoadMarkingCand_1(:, 2), 3), 'go', 'linewidth', 2, 'markersize', 20)
% fscatter3(pc.x, pc.y, pc.z, pc.intensity, cmap); % point cloud intensity image

%% Grow road markings from candidates

% ??How robust the marking extraction is against variations in the scanning
% geometry? Painting width, other? Most of the time trajectory is parallel
% to the road direction. Therefore it can be used to correct the width of
% the painting if the profile is not perpendicular to the driving
% direction. So if you are extracting candidates from a profile, you can
% give the direction of the traj as an input.


ticID = tic;


% Curve from positive jumps 
[curves_posIntJumps, lenghts_curves_posIntJumps, markings_ins_pc] = ...
    f_growMarkings_overProfs(roadMarkingCand_2, Xyzti, ins_prof_pc, par_markings);

n_curves_posIntJumps = length( curves_posIntJumps );

t_elapsed_markingRetr = toc(ticID);
disp(['Marking retrieval: ', num2str(t_elapsed_markingRetr), ' seconds.'])

%% Plot extracted curves

% ind_fig = ind_fig + 1;
% f_initFig(ind_fig, 'k')
% fscatter3_edit_Joona(pc.x, pc.y, pc.z, pc.intensity, cmap); % point cloud intensity image
% for ind_marking = 1 : length(curves_posIntJumps)
%     ins_pointsOneMarking_pc = curves_posIntJumps{ind_marking}; % indices of the points on the curve
%     plot3(Xyzti(ins_pointsOneMarking_pc, 1), Xyzti(ins_pointsOneMarking_pc, 2), Xyzti(ins_pointsOneMarking_pc, 3), 'linewidth', 3, 'color', [0.68, 0.92, 1]); % [0.68, 0.92, 1]
% end

%% Fill gaps on markings  

ticID = tic;

markingsFilled_ins_pc = f_fillGaps_roadMarkings(markings_ins_pc, ins_prof_pc, Xyzti, par_markings);

% elapsed time
t_elapsed_fillMarkings = toc(ticID);
disp(['Fill marking holes: ', num2str(t_elapsed_fillMarkings), ' seconds.'])

%% Stoppa här

% % save rest important workspace variables
% save 'E:\Combat_roadModelling_data\wstemp_161124_162516_1b.mat'
% 
% return


%% Plot filled markings; each marking with a different colour

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'w')
% plot3(pc.x, pc.y, pc.z, '.', 'color', 0.7*[1 1 1]); % point cloud 
for ind_marking = 1 : length(markingsFilled_ins_pc)
    ins_pointsOneMarking_pc = markingsFilled_ins_pc{ind_marking}; % indices of the points on the curve
    plot3(Xyzti(ins_pointsOneMarking_pc, 1), Xyzti(ins_pointsOneMarking_pc, 2), Xyzti(ins_pointsOneMarking_pc, 3), '.')
end

%% Classify road markings

% ??parannusehdotus: Hae trajektorista lähimmät pisimmät viivat, yksi kummaltakin puolelta
% (ensin lähin piste trajektorilta, sitten tästä molempiin suuntiin
% lähimmät viivat) ??tämä toimii simppelissä tapauksessa, kun ei ole useita yhtenäisiä
% reunaviivoja (esim. liittymä sivulla). Luokittele nämä reunaviivoiksi

c_curves_posIntJumps = nan( n_curves_posIntJumps, 1 ); % class labels for the curves
for ind_marking = 1 : n_curves_posIntJumps
    
    % indices of the points on the curve
    ins_pointsOneMarking_pc = markingsFilled_ins_pc{ind_marking};
    
    % classify
    if lenghts_curves_posIntJumps(ind_marking) >= par_markings.thre_length_marking_border
        % border marking
        c_curves_posIntJumps(ind_marking) = classLabel_marking_long;
        % pc.classification( ins_pointsOneMarking_pc ) = classLabel_marking_long;
    elseif lenghts_curves_posIntJumps(ind_marking) >= par_markings.thre_length_marking_centre
        % centre marking
        c_curves_posIntJumps(ind_marking) = classLabel_marking_short;
        pc.classification( ins_pointsOneMarking_pc ) = classLabel_marking_short;
    end
end

% Retain only curves classified as markings
li_curvesPosIntJump_marking = ~isnan( c_curves_posIntJumps );
markingsFilled_ins_pc = markingsFilled_ins_pc( li_curvesPosIntJump_marking );
markings_ins_pc = markings_ins_pc( li_curvesPosIntJump_marking );
curves_posIntJumps = curves_posIntJumps( li_curvesPosIntJump_marking );
lenghts_curves_posIntJumps = lenghts_curves_posIntJumps( li_curvesPosIntJump_marking );
c_curves_posIntJumps = c_curves_posIntJumps( li_curvesPosIntJump_marking );
n_markings = sum( li_curvesPosIntJump_marking );

%% Merge border markings and classify on each side of the road

ticID = tic;

% Merge border marking (bms) segments 
K_bms = ... 
    f_mergeMarkingSegments( ...
    markingsFilled_ins_pc(c_curves_posIntJumps == classLabel_marking_long), ...
    lenghts_curves_posIntJumps(c_curves_posIntJumps == classLabel_marking_long), ...
    Xyzti);

% Direction of the trajectory
Dir_traj = f_trajDir(Traj(:, [ci_Traj_x, ci_Traj_y]));

% Find one point from each segment
[segIds_bms, ins_markingPoints_uniqueSeg] = unique( K_bms(:, 2) );
ins_pc_onePointOnEachMarking = K_bms( ins_markingPoints_uniqueSeg, 1 );
n_bms = length( ins_pc_onePointOnEachMarking ); % number of bms

% Find closest trajectory point for each selected point on each curve

% ??if trajectory crosses the marking, this idea does not work 

ins_trajPts_closest2marking = nan( size( ins_pc_onePointOnEachMarking ) );
for ind_borderMarking = 1 : n_bms
    ind_pc_marking = ins_pc_onePointOnEachMarking( ind_borderMarking );
    ins_trajPts_closest2marking(ind_borderMarking) = ...
        f_closestTrajPoint( Xyzti(ind_pc_marking, 4), Traj(:, ci_Traj_t) );
end

% Find on which side of the traj the marking is
P_traj2marking = Xyzti(ins_pc_onePointOnEachMarking, 1:2) - ... vectors pointing from traj to marking
    Traj(ins_trajPts_closest2marking, [ci_Traj_x, ci_Traj_y]);

% scale vectors to unit length
norms_rows = sqrt( sum( P_traj2marking.^2, 2 ) ); % norms of the rows
P_traj2marking = P_traj2marking ./ f_repmat_mtx_fast( norms_rows, 1, size( P_traj2marking, 2 ) );

% Calculate cross product. The signs should tell on which side of the traj
% the marking is
crossProd_bms = cross( [P_traj2marking, zeros(n_bms, 1)], ...
    [Dir_traj(ins_trajPts_closest2marking, :), zeros(n_bms, 1)] );

% Classify markings as 'left side' or 'right side' of the trajectory
li_bms_leftSideOfTraj = crossProd_bms(:, 3) <= 0;
ins_pc_borderMarkings_leftSide = K_bms( ismember( K_bms(:, 2) , segIds_bms(li_bms_leftSideOfTraj) ), 1);
ins_pc_borderMarkings_rightSide = K_bms( ismember( K_bms(:, 2) , segIds_bms(~li_bms_leftSideOfTraj) ), 1);

t_elapsed_mergeBorderMarkings = toc(ticID);
disp(['Border marking merging: ', num2str(t_elapsed_mergeBorderMarkings), ' seconds.'])

%% LAS file classification

pc.classification(ins_pc_borderMarkings_leftSide) = classLabel_marking_border_left;
pc.classification(ins_pc_borderMarkings_rightSide) = classLabel_marking_border_right;

%% Plot merged border markings 

% % each segment separately
% 
% ind_fig = ind_fig + 1;
% f_initFig(ind_fig, 'w')
% % plot3(pc.x, pc.y, pc.z, '.', 'color', 0.7*[1 1 1]); % point cloud 
% segIds_borderMarkingSegs = unique( K_bms(:, 2) ); % segment Ids
% for ind_mbs = 1 : length(segIds_borderMarkingSegs)
%     mbs_segId = segIds_borderMarkingSegs(ind_mbs);
%     ins_pointsOneMarking_pc = K_bms( K_bms(:, 2) == mbs_segId , 1); % indices of the points on the curve
%     plot3(Xyzti(ins_pointsOneMarking_pc, 1), Xyzti(ins_pointsOneMarking_pc, 2), Xyzti(ins_pointsOneMarking_pc, 3), '.')
% end
% 
% % one selected point on each segment
% plot3(Xyzti(ins_pc_onePointOnEachMarking, 1), Xyzti(ins_pc_onePointOnEachMarking, 2), Xyzti(ins_pc_onePointOnEachMarking, 3), 'diamond', 'linewidth', 2, 'markersize', 20) 

%% Plot right and left border marking points

% two BMSs; one on the right side, the other on the left side of the traj

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'w')
plot3(Xyzti(ins_pc_borderMarkings_leftSide, 1), Xyzti(ins_pc_borderMarkings_leftSide, 2), Xyzti(ins_pc_borderMarkings_leftSide, 3), '.')
plot3(Xyzti(ins_pc_borderMarkings_rightSide, 1), Xyzti(ins_pc_borderMarkings_rightSide, 2), Xyzti(ins_pc_borderMarkings_rightSide, 3), '.')
plot3(Traj(1:100:end, ci_Traj_x), Traj(1:100:end, ci_Traj_y), Traj(1:100:end, ci_Traj_z))

%% Find lane separator markings

% Find short marking. Calculate distance to the left border. Find peak.
% Select the segmen as a seed whose distance is closest to the peak.

% ??Other option: Find long enough seed using Ransac. ??A function is
% started in file f_centreMarkingSeed.m (based on power line extraction
% code, joka on jo toimivaksi havaittu. Siis riittää, että muokkaat sitä
% tähän tarkoitukseen sopivaksi). Start from line "logInsLeft_line_ransac =
% f_ransac_line_nD_10012013(P_linear_hor(ins_left, 1:2), thre_numPts_line,
% thre_normDist, max_numIter_ransac); % do not use the direction of the
% points"    

% ??korvaa yllä (jos tarvetta) border marking bm:llä muuttujien nimissä


ticID = tic;

% Merge short marking segments to get lane separator markings (lsm) 
li_pc_shortMarkings = pc.classification == classLabel_marking_short;
K_lsm = ... 
    f_mergeMSlane( ...
    markingsFilled_ins_pc(c_curves_posIntJumps == classLabel_marking_short), ...
    lenghts_curves_posIntJumps(c_curves_posIntJumps == classLabel_marking_short), ...
    Xyzti, li_pc_shortMarkings, ins_prof_pc, ins_pc_borderMarkings_leftSide);

t_elapsed_laneSeparator = toc(ticID);
disp(['Lane separator markings: ', num2str(t_elapsed_laneSeparator), ' seconds.'])

%% LAS file classification

pc.classification(K_lsm(:, 1)) = classLabel_marking_laneSeparator;

%% Plot lane separator markings (merged short markings)

% % each segment separately
% 
% ind_fig = ind_fig + 1;
% f_initFig(ind_fig, 'w')
% for ind_seg = 1 : max( K_lsm(:, 2) )
%     li_pc_seg = K_lsm( K_lsm(:, 2) == ind_seg, 1 );
%     plot3(Xyzti(li_pc_seg, 1), Xyzti(li_pc_seg, 2), Xyzti(li_pc_seg, 3), '.')
% end
% plot3(Xyzti(1:30:end, 1), Xyzti(1:30:end, 2), Xyzti(1:30:end, 3), '.', 'markersize', 2, 'color', 0.75*[1 1 1])

%% LAS file classification 

% Remove temporary point classifications from marking extraction

% ??tee myöhemmin niin, ettet laita tällaisia väliaikaisia luokitteluja
% lassiin ollenkaan; vain loogisina indekseinä tähän skriptiin

li_pc_shortMarkings = pc.classification == classLabel_marking_short;
pc.classification(li_pc_shortMarkings) = classLabel_unclassified;
li_pc_longMarkings = pc.classification == classLabel_marking_long;
pc.classification(li_pc_longMarkings) = classLabel_unclassified;

%% plot all markings; colour based on class label

% ind_fig = ind_fig + 1;
% f_initFig(ind_fig, 'w')
% 
% li_pc_bm_left = pc.classification == classLabel_marking_border_left;
% li_pc_bm_right = pc.classification == classLabel_marking_border_right;
% li_pc_marking_laneSeparator = pc.classification == classLabel_marking_laneSeparator;
% li_pc_unclassif = pc.classification == classLabel_unclassified;
% 
% plot3(pc.x(li_pc_bm_left), pc.y(li_pc_bm_left), pc.z(li_pc_bm_left), '.')
% plot3(pc.x(li_pc_bm_right), pc.y(li_pc_bm_right), pc.z(li_pc_bm_right), '.')
% plot3(pc.x(li_pc_marking_laneSeparator), pc.y(li_pc_marking_laneSeparator), pc.z(li_pc_marking_laneSeparator), '.')
% 
% legend('Left border marking', 'Right border marking', 'Lane separator marking')
% 
% plot3(pc.x(li_pc_unclassif), pc.y(li_pc_unclassif), pc.z(li_pc_unclassif), '.', 'markersize', 2, 'color', 0.9*[1 1 1])

%% Classify area of traffic lanes profile-wise

% That is, the carriageway without the shoulders, i.e., the area between
% the inner edges of the boundary markings 

% ??Tsekkaa ao. funktiosta vielä,
% tuleeko negatiivisia pistemääriä (if lause fkt:n sisällä). Poista lopuksi
% pistepilvi inputista, se oli vain testausta varten.

ticID = tic;
[ins_prof_lanePoints, ins_lanePoints_pc] = ...
        f_classifyLaneAreaPoints(ins_prof_pc, {ins_pc_borderMarkings_leftSide, ins_pc_borderMarkings_rightSide}, ...
        Xyzti);
t_elapsed_laneArea = toc(ticID);
disp(['Lane area retrieval: ', num2str(t_elapsed_laneArea), ' seconds.'])
    
%% LAS file classification

li_pc_laneArea = false( n_pc, 1 ); % li for pc; true indicates that the point is in the lane area (excluding more spesific classes such as centre marking)
li_pc_laneArea(ins_lanePoints_pc) = true;
li_pc_laneArea( pc.classification == classLabel_marking_laneSeparator ) = false; % remove lane separator markings
pc.classification(li_pc_laneArea) = classLabel_laneArea;

%% Plot classified points

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'w')

li_pc_marking_laneSeparator = pc.classification == classLabel_marking_laneSeparator;
li_pc_bm_left = pc.classification == classLabel_marking_border_left;
li_pc_bm_right = pc.classification == classLabel_marking_border_right;
ins_pc_laneArea = find( pc.classification == classLabel_laneArea );
ins_pc_unclassif = find( pc.classification == classLabel_unclassified );

plot3(pc.x(li_pc_marking_laneSeparator), pc.y(li_pc_marking_laneSeparator), pc.z(li_pc_marking_laneSeparator), '.')
plot3(pc.x(li_pc_bm_left), pc.y(li_pc_bm_left), pc.z(li_pc_bm_left), '.')
plot3(pc.x(ins_pc_laneArea(1:10:end)), pc.y(ins_pc_laneArea(1:10:end)), pc.z(ins_pc_laneArea(1:10:end)), '.')
plot3(pc.x(li_pc_bm_right), pc.y(li_pc_bm_right), pc.z(li_pc_bm_right), '.')
plot3(pc.x(ins_pc_unclassif(1:30:end)), pc.y(ins_pc_unclassif(1:30:end)), pc.z(ins_pc_unclassif(1:30:end)), '.', 'markersize', 2, 'color', 0.8*[1 1 1])


%% Improve traffic lane point retrieval using lane edges

ticID = tic;

% select lane edge points with a lower resolution
res_laneEdge = 10; % (metres) minimum resolution ("pixel size") of the lane edge ??parametriksi inputtiin
res_laneEdge_nProfs = res_laneEdge / par.ul_profSpacing; % resolution in number of profiles

% lane edges with the desired resolution
ins_pc_laneEdge1_sparse = vertcat(ins_prof_lanePoints{1:res_laneEdge_nProfs:end, 1}); % indices (pc) of the start points of the lane
ins_pc_laneEdge2_sparse = vertcat(ins_prof_lanePoints{1:res_laneEdge_nProfs:end, 2}); % indices (pc) of the end points of the lane

% A polygon for the lane points
ins_switchOrder = sort(1 : length(ins_pc_laneEdge2_sparse), 'descend'); % aid indices to reorder other lane edge
ins_pc_laneEdgePolygon = [ins_pc_laneEdge1_sparse; ins_pc_laneEdge2_sparse( ins_switchOrder )];

% li for pc; true indicates that the point is unclassified
ins_pc_unclassif = find( pc.classification == classLabel_unclassified );

% ??miksi polygoni ei ulotu blokin alkuun?


% Retrieve points inside the polygon
[li_unclassifPts_lane, li_unclassifPts_laneEdge] = inpolygon(Xyzti(ins_pc_unclassif, 1), Xyzti(ins_pc_unclassif, 2), ...
    Xyzti(ins_pc_laneEdgePolygon, 1), ...
    Xyzti(ins_pc_laneEdgePolygon, 2));


% linear indices (pc) of the new lane candidates
ins_pc_newLaneCand = [ ins_pc_unclassif(li_unclassifPts_lane); ins_pc_unclassif(li_unclassifPts_laneEdge) ];

% select only points which are flat and horizontal (not linear)
pointAttr_newLaneCand = f_pointAttr( Xyzti(ins_pc_newLaneCand, 1:3), 4, par.rn_pointAttr, false); % attributes of the candidates points
li_newLaneCand_flat = pointAttr_newLaneCand.flatness >= par.thre_flatness_roadSurf; % li for candidates points; true indicates that the point is flat
li_newLaneCand_hor = pointAttr_newLaneCand.alpha_vert <= par.ul_alphaVert_hor; % li for candidates points; true indicates that the point is horizontal
li_newLaneCand_linear = pointAttr_newLaneCand.linearness >= par.thre_linearity; % li for candidates points; true indicates that the point is linear
ins_pc_newLaneCand = ins_pc_newLaneCand(li_newLaneCand_flat & li_newLaneCand_hor & ~li_newLaneCand_linear);

% remove points above ground
F = scatteredInterpolant( ... interpolant to be used as a dem
    Xyzti(ins_pc_laneEdgePolygon, 1), ...
    Xyzti(ins_pc_laneEdgePolygon, 2), ...
    Xyzti(ins_pc_laneEdgePolygon, 3));
z_ground_newLaneCand = F(Xyzti(ins_pc_newLaneCand, 1:2)); % ground level for the new candidates
li_newLaneCand_air = Xyzti(ins_pc_newLaneCand, 3) - z_ground_newLaneCand > 0.1; % li for the candidates; true indicates that the point is above ground ??parametriksi ylös sektioon
ins_pc_newLaneCand = ins_pc_newLaneCand(~li_newLaneCand_air); % remove above ground candidates

t_elapsed_improveLaneArea = toc(ticID);
disp(['Improve lane area: ', num2str(t_elapsed_improveLaneArea), ' seconds.'])

%% LAS file classification

pc.classification(ins_pc_newLaneCand) = classLabel_laneArea;

%% Find search space (SP) for the road surface edge (RSE)

% take, e.g., 4 metres from the border marking towards forest, that is,
% roughly the shoulder and verge (sv) area.

% ??vajaasta profiilista (eka tai vika) ei ole löytynyt kaistapisteitä,
% jolloin ne luokitellaan alla piennarpisteiksi (voi olla vanhaa tietoa...)

ticID = tic;

% Parameter for finding all points that are closer than 50 cm to the lane
% edges. The road edge cannot be in this area.
w_laneAreaEdgeNeigh = 0.5; % % parameters ??lisää ylös omaan sektioon

% directions of the lane area edges, scaled to unit length
Dir_laneEdge1 = Xyzti( ins_pc_laneEdge1_sparse( 2 : end ), 1:2 ) - Xyzti( ins_pc_laneEdge1_sparse( 1 : end - 1 ), 1:2 );
Dir_laneEdge1(end + 1, :) = Dir_laneEdge1(end, :); % the last point; use same direction as in the second last
norms_rows = sqrt( sum( Dir_laneEdge1.^2, 2 ) ); % norms of the rows
li_nonZeroNorm = norms_rows > 0; % find non-zero vectors
Dir_laneEdge1(li_nonZeroNorm, :) = Dir_laneEdge1(li_nonZeroNorm, :) ./ f_repmat_mtx_fast( norms_rows(li_nonZeroNorm), 1, size( Dir_laneEdge1, 2 ) );
Dir_laneEdge2 = Xyzti( ins_pc_laneEdge2_sparse( 2 : end, : ), 1:2 ) - Xyzti( ins_pc_laneEdge2_sparse( 1 : end - 1, : ), 1:2 );
Dir_laneEdge2(end + 1, :) = Dir_laneEdge2(end, :); % the last point; use same direction as in the second last
norms_rows = sqrt( sum( Dir_laneEdge2.^2, 2 ) ); % norms of the rows
li_nonZeroNorm = norms_rows > 0; % find non-zero vectors
Dir_laneEdge2(li_nonZeroNorm, :) = Dir_laneEdge2(li_nonZeroNorm, :) ./ f_repmat_mtx_fast( norms_rows(li_nonZeroNorm), 1, size( Dir_laneEdge2, 2 ) );

% Find vectors orthogonal to the edge direction
Xy_edge_sv1 = nan( size( Dir_laneEdge1 ) ); % shoulder and verge (sv) edge estimates
Xy_close2laneEdge1 = nan( size( Dir_laneEdge1 ) ); % points close to the lane edges (border marking)
Xy_edge_sv2 = nan( size( Dir_laneEdge2 ) ); % shoulder and verge (sv) edge estimates
Xy_close2laneEdge2 = nan( size( Dir_laneEdge2 ) ); % points close to the lane edges (border marking)

% ---------------
% First edge

% Test with one vector if the orthogonal vector point towards the road
% centre line

% ??toivotaan, että null antaa aina samansuuntaisen vektorin
ind_testPoint = 6; % 12 before 3.7.2017
v = null(Dir_laneEdge1(ind_testPoint, :))'; % orthogonal vector
p_test = Xyzti( ins_pc_laneEdge1_sparse(ind_testPoint), 1:2 ) + v;
tf_inLaneArea1 = ...
    inpolygon(p_test(1), p_test(2), ...
    Xyzti(ins_pc_laneEdgePolygon, 1), ...
    Xyzti(ins_pc_laneEdgePolygon, 2));

for ind_edgePoint = 1 : size( Xy_edge_sv1, 1 )
    v = null(Dir_laneEdge1(ind_edgePoint, 1:2))'; % orthogonal vector
    v = v / norm(v);
    if ~tf_inLaneArea1
        Xy_edge_sv1(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge1_sparse(ind_edgePoint), 1:2 ) + par_rse.width_verge_shoulder * v;
        
        
        Xy_close2laneEdge1(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge1_sparse(ind_edgePoint), 1:2 ) + w_laneAreaEdgeNeigh * v;
        
        
    else
        Xy_edge_sv1(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge1_sparse(ind_edgePoint), 1:2 ) - par_rse.width_verge_shoulder * v;
        
        Xy_close2laneEdge1(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge1_sparse(ind_edgePoint), 1:2 ) - w_laneAreaEdgeNeigh * v;
        
    end
end

% ---------------
% Second edge

% Test with one vector if the orthogonal vector point towards the road
% centre line
ind_testPoint = 6; % 12 before 3.7.2017
v2 = null(Dir_laneEdge2(ind_testPoint, :))';
p_test = Xyzti( ins_pc_laneEdge2_sparse(ind_testPoint), 1:2 ) + v2;
tf_inLaneArea2 = ...
    inpolygon(p_test(1), p_test(2), ...
    Xyzti(ins_pc_laneEdgePolygon, 1), ...
    Xyzti(ins_pc_laneEdgePolygon, 2));

for ind_edgePoint = 1 : size( Xy_edge_sv2, 1 )
    v = null(Dir_laneEdge2(ind_edgePoint, 1:2))'; % orthogonal vector
    v = v / norm(v);
    if ~tf_inLaneArea2
        Xy_edge_sv2(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge2_sparse(ind_edgePoint), 1:2 ) + par_rse.width_verge_shoulder * v;
        
        
        Xy_close2laneEdge2(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge2_sparse(ind_edgePoint), 1:2 ) + w_laneAreaEdgeNeigh * v;
        
        
    else
        Xy_edge_sv2(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge2_sparse(ind_edgePoint), 1:2 ) - par_rse.width_verge_shoulder * v;
        
        
        Xy_close2laneEdge2(ind_edgePoint, :) = ...
            Xyzti( ins_pc_laneEdge2_sparse(ind_edgePoint), 1:2 ) - w_laneAreaEdgeNeigh * v;
        
    end
end

% Construct polygons for the shoulder and verge (sv) areas
ins_switchOrder = sort(1 : size(Xy_edge_sv1, 1), 'descend'); % aid indices to reorder other sv edge
Xy_polygon_sv1 = [ Xyzti( ins_pc_laneEdge1_sparse, 1:2 ); Xy_edge_sv1(ins_switchOrder, :) ];
ins_switchOrder = sort(1 : size(Xy_edge_sv2, 1), 'descend'); % aid indices to reorder other sv edge
Xy_polygon_sv2 = [ Xyzti( ins_pc_laneEdge2_sparse, 1:2 ); Xy_edge_sv2(ins_switchOrder, :) ];

% Construct polygons for the points very close to the lane area edge
ins_switchOrder = sort(1 : size(Xy_close2laneEdge1, 1), 'descend'); % aid indices to reorder
Xy_polygon_close2laneEdge1 = [ Xyzti( ins_pc_laneEdge1_sparse, 1:2 ); Xy_close2laneEdge1(ins_switchOrder, :) ];
ins_switchOrder = sort(1 : size(Xy_close2laneEdge2, 1), 'descend'); % aid indices to reorder
Xy_polygon_close2laneEdge2 = [ Xyzti( ins_pc_laneEdge2_sparse, 1:2 ); Xy_close2laneEdge2(ins_switchOrder, :) ];


% Retrieve points inside the polygons 

% ??each run takes about 15 minutes (50 million points, 1000 vertices)

% linear indices (pc) for unclassified points
ins_pc_unclassified = find( pc.classification == classLabel_unclassified );


% sv1
[li_unclassifPts_sv_1, li_unclassifPts_edge_sv1] = ...
    inpolygon(Xyzti(ins_pc_unclassified, 1), Xyzti(ins_pc_unclassified, 2), ...
    Xy_polygon_sv1(:, 1), ...
    Xy_polygon_sv1(:, 2));

% sv2
[li_unclassifPts_sv_2, li_unclassifPts_edge_sv2] = ...
    inpolygon(Xyzti(ins_pc_unclassified, 1), Xyzti(ins_pc_unclassified, 2), ...
    Xy_polygon_sv2(:, 1), ...
    Xy_polygon_sv2(:, 2));

% close to border marking 1
[li_unclassifPts_close2laneEdge1, li_unclassifPts_edge_lane1] = ...
    inpolygon(Xyzti(ins_pc_unclassified, 1), Xyzti(ins_pc_unclassified, 2), ...
    Xy_polygon_close2laneEdge1(:, 1), ...
    Xy_polygon_close2laneEdge1(:, 2));

% close to border marking 2
[li_unclassifPts_close2laneEdge2, li_unclassifPts_edge_lane2] = ...
    inpolygon(Xyzti(ins_pc_unclassified, 1), Xyzti(ins_pc_unclassified, 2), ...
    Xy_polygon_close2laneEdge2(:, 1), ...
    Xy_polygon_close2laneEdge2(:, 2));

% Select road surface edge (rse) search space (sp) from the point cloud,
% that is, shoulder and verge without the close neighbourhood of the border
% marking 
li_unclassifPts_sp_rse_1 = ( li_unclassifPts_sv_1 | li_unclassifPts_edge_sv1 ) & ~(li_unclassifPts_close2laneEdge1| li_unclassifPts_edge_lane1);
li_unclassifPts_sp_rse_2 = ( li_unclassifPts_sv_2 | li_unclassifPts_edge_sv2 ) & ~(li_unclassifPts_close2laneEdge2| li_unclassifPts_edge_lane2);
ins_pc_sp_rse_1 = ins_pc_unclassified( li_unclassifPts_sp_rse_1 ); % indices (pc) of the points in the search space (sp) for road surface edge (rse) one.
ins_pc_sp_rse_2 = ins_pc_unclassified( li_unclassifPts_sp_rse_2 ); % indices (pc) of the points in the search space (sp) for road surface edge (rse) two.

t_elapsed_sv = toc(ticID);


% Find flat horizontal points (exclude also linear points) from the
% search space (sp) for road surface edge (rse), that is, points whose
% local neighbourhood is close to planar. This removes fences etc.

ticID = tic; 

Xyz_sp_rse1 = [ pc.x(ins_pc_sp_rse_1), pc.y(ins_pc_sp_rse_1), pc.z(ins_pc_sp_rse_1) ]; % points on the first shoulder and verge
Xyz_sp_rse2 = [ pc.x(ins_pc_sp_rse_2), pc.y(ins_pc_sp_rse_2), pc.z(ins_pc_sp_rse_2) ]; % points on the second shoulder and verge
pointAttr_sp_rse1 = f_pointAttr( Xyz_sp_rse1, 4, par_rse.rn_pointAttr, false); % point attributes of the first shoulder
pointAttr_sp_rse2 = f_pointAttr( Xyz_sp_rse2, 4, par_rse.rn_pointAttr, false); % point attributes of the second shoulder
li_sp_rse1pts_flat = pointAttr_sp_rse1.flatness >= par_rse.thre_flatness_roadSurf; % li for points in the first shoulder; true indicates that the point is flat
li_sp_rse2pts_flat = pointAttr_sp_rse2.flatness >= par_rse.thre_flatness_roadSurf; % li for points in the second shoulder; true indicates that the point is flat
li_sp_rse1pts_hor = pointAttr_sp_rse1.alpha_vert <= par_rse.ul_alphaVert_hor; % li for points in the first shoulder; true indicates that the point is horizontal
li_sp_rse2pts_hor = pointAttr_sp_rse2.alpha_vert <= par_rse.ul_alphaVert_hor; % li for points in the second shoulder; true indicates that the point is horizontal
li_sp_rse1pts_linear = pointAttr_sp_rse1.linearness >= par_rse.thre_linearity; % li for points in the first shoulder; true indicates that the point is linear
li_sp_rse2pts_linear = pointAttr_sp_rse2.linearness >= par_rse.thre_linearity; % li for points in the second shoulder; true indicates that the point is linear
li_pc_sp_rse1_flatHor = false( n_pc, 1 ); % li for pc; true indicates that the point is flat and horizontal (not linear) and belongs to the search space (sp) for first road surface edge (rse)
li_pc_sp_rse2_flatHor = false( n_pc, 1 ); % li for pc; true indicates that the point is flat and horizontal (not linear) and belongs to the search space (sp) for second road surface edge (rse)
li_pc_sp_rse1_flatHor( ins_pc_sp_rse_1( li_sp_rse1pts_flat & li_sp_rse1pts_hor & ~li_sp_rse1pts_linear ) ) = true;
li_pc_sp_rse2_flatHor( ins_pc_sp_rse_2( li_sp_rse2pts_flat & li_sp_rse2pts_hor & ~li_sp_rse2pts_linear ) ) = true;

% Profile indices of the shoulder+verge points
% ins_prof_shoulderVerge1pts = ins_pc_prof( li_pc_shoulderVerge1_flatHor );
% ins_prof_shoulderVerge2pts = ins_pc_prof( li_pc_shoulderVerge2_flatHor );

t_elapsed_flatPts_sv = toc(ticID); 
disp(['Search space for road surface edges: ', num2str(t_elapsed_flatPts_sv + t_elapsed_sv), ' seconds.'])


%% Save workspace to a mat file 

% save 'E:\Combat_roadModelling_data\wstemp_125641_6a.mat'


%% put intensity in the fourth column

% ??make a Xyzti nx5 matrix later

Xyzi(:, 1) = pc.x;
Xyzi(:, 2) = pc.y;
Xyzi(:, 3) = pc.z;
Xyzi(:, 4) = pc.intensity;

%% find road surface edge candidates from the profiles

ticID = tic; 

roadEdgeCands = cell( n_profs, 1 );

for ind_prof = 1 : n_profs
    
    % ind_prof
    
    % sv points on the current profile; these are sorted based on time
    % stamps
    li_pc_sv1prof = ins_prof_pc == ind_prof & li_pc_sp_rse1_flatHor; % li for pc; true indicates that the point is on the sv1 of the current profile
    li_pc_sv2prof = ins_prof_pc == ind_prof & li_pc_sp_rse2_flatHor; % the same for sv2
    ins_pc_sv1prof = find( li_pc_sv1prof ); % linear indices of the points (pc) which are on the sv1 of the current profile
    ins_pc_sv2prof = find( li_pc_sv2prof ); % the same for sv2
    Xyzi_prof_sv1 = Xyzi( li_pc_sv1prof, : ); % the corresponding points for sv1
    Xyzi_prof_sv2 = Xyzi( li_pc_sv2prof, : ); % the corresponding points for sv2
    
    % Check if the sv point matrices are empty
    if ~isempty( Xyzi_prof_sv1 ) && ~isempty( Xyzi_prof_sv2 ) % ??what if only one is empty?
        
        % find the road edge candidates
        
        % disp('sv1')
        cand_sv1 = f_roadEdgeCandPts(Xyzi_prof_sv1, par_rse); % Find road edge candicate points from sv1
        % disp('sv2')
        cand_sv2 = f_roadEdgeCandPts(Xyzi_prof_sv2, par_rse); % Find road edge candicate points from sv2
        
        % select pos and neg jumps correctly for both sides
        ind_pc_laneEdge1prof = ins_prof_lanePoints{ind_prof, 1}; % index of the start point (pc) of the lane
        ind_pc_laneEdge2prof = ins_prof_lanePoints{ind_prof, 2}; % index of the end point (pc) of the lane
        
        if ~isempty( ind_pc_laneEdge1prof ) && ~isempty( ind_pc_laneEdge2prof )
            
            % ??mahdollisesti voi hyväksyä vain toisenkin, mutta vaatii
            % muutosta kodiin
            
            if max( ins_pc_sv1prof ) > ind_pc_laneEdge1prof
                
                % select negative jump for sv1 and positivie jumps for sv2
                ins_pc_heightJumps_sv1 = ins_pc_sv1prof( cand_sv1.li_ptsProf_negHeightJump ); % indices of the points (pc) corresponding to the height jumps on sv1
                ins_pc_heightJumps_sv2 = ins_pc_sv2prof( cand_sv2.li_ptsProf_posHeightJump ); % indices of the points (pc) corresponding to the height jumps on sv2
                
            else
                
                % select negative jump for sv2 and positivie jumps for sv1
                ins_pc_heightJumps_sv1 = ins_pc_sv1prof( cand_sv1.li_ptsProf_posHeightJump ); % indices of the points (pc) corresponding to the height jumps on sv1
                ins_pc_heightJumps_sv2 = ins_pc_sv2prof( cand_sv2.li_ptsProf_negHeightJump ); % indices of the points (pc) corresponding to the height jumps on sv2
                
            end
            
            % Find the candidates that are closest to the lane edges
            
            
            % ??improve later with additional checks. ??does
            % this idea (closest to the lane edge) always work?
            
            if ~isempty( ins_pc_heightJumps_sv1 )
                ind_sv1pts_closest2edge1 = f_match_1D(ind_pc_laneEdge1prof, ins_pc_heightJumps_sv1); % index of the height jump on sv1 that is closest to the lane edge 1
                ind_pc_closest2edge1 = ins_pc_heightJumps_sv1( ind_sv1pts_closest2edge1 ); % corresponding index for pc
            else
                ind_pc_closest2edge1 = [];
            end
            
            if ~isempty( ins_pc_heightJumps_sv2 )
                ind_sv2pts_closest2edge2 = f_match_1D(ind_pc_laneEdge2prof, ins_pc_heightJumps_sv2); % index of the height jump on sv2 that is closest to the lane edge 2
                ind_pc_closest2edge2 = ins_pc_heightJumps_sv2( ind_sv2pts_closest2edge2 ); % corresponding index for pc
            else
                ind_pc_closest2edge2 = [];
            end
            
            roadEdgeCands{ ind_prof } = [ ind_pc_closest2edge1; ind_pc_closest2edge2 ];
            
        end
    end
end

t_elapsed_rseCand = toc(ticID);
disp(['Road surface edge candidates: ', num2str(t_elapsed_rseCand), ' seconds.'])

%% Find curves from road edge candidates

ticID = tic;

[curves_roadEdgeCands, lenghts_curves_roadEdgeCands, ci_cands_roadEdge] = ...
    f_growCurve_overProfs2(roadEdgeCands, Xyzti, ins_prof_pc, par_rse);

% elapsed time
t_elapsed_rseCurves = toc(ticID);
disp(['Road surface edge curves: ', num2str(t_elapsed_rseCurves), ' seconds.'])


%% Classify road surface edge points

% these points are in the same area as shoulders and verge but are
% classified as edges

ins_pc_roadSurfEgde = vertcat( curves_roadEdgeCands{:} ); % linear indices (pc) of the edge points
li_pc_roadSurfEdge = false( n_pc, 1 ); % the corresponding logical indices (pc)
li_pc_roadSurfEdge(ins_pc_roadSurfEgde) = true;
pc.classification(li_pc_roadSurfEdge) = classLabel_roadSurfEdge;


%% stoppa här

% % save rest important workspace variables
% save('E:\Combat_roadModelling_data\wstemp_125641_6b.mat', ...
%     'roadEdgeCands', 't_elapsed_rseCand', 'curves_roadEdgeCands', ...
%     'lenghts_curves_roadEdgeCands', 'ci_cands_roadEdge', 't_elapsed_rseCurves')
% 
% % write las file
% mat2las(pc, '-fgi_scale 0.001 0.001 0.001 -o E:\Combat_roadModelling_data\RoadDataRoamerR3\VT1\160421_125641_VUX-1-HA_etrs_edges_6.las');
% 
% return


%% Fill gaps on the curves 

% ??käyrän täytössä tulee hieman sama ongelma jota jo aikanaan mietittiin
% Anteron kanssa. Kun haetaan peilin jakson perusteella samalla kohtaa olevaa
% pistettä seuraavalta profiililta, piste alkaa ajelehtia profiilia pitkin.
% Parempi olisi ympätä täyttö osaksi kasvatusta, sillä siinä estimoidaan
% suora, jonka avulla voidaan saada lähinnä käyrää (suoraa) olevat pisteet
% helposti.

curves_roadEdgeCands = f_fillGaps_roadSurfEdge(curves_roadEdgeCands, ins_prof_pc, Xyzti, par_markings);

%% Plot curves from road surface edge candidates

% % testing
ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'k')

cmap = colormap('parula');
fscatter3(pc.x(1:50:end), pc.y(1:50:end), pc.z(1:50:end), pc.intensity(1:50:end), cmap);

for ind_curve = 1 : length( curves_roadEdgeCands )
    ins_pc_curve = curves_roadEdgeCands{ind_curve}; % indices of the points on the curve
    plot3(Xyzti(ins_pc_curve, 1), Xyzti(ins_pc_curve, 2), Xyzti(ins_pc_curve, 3), 'w-', 'linewidth', 2)
end

% plot3(Xyzti(vertcat(roadEdgeCands{ : }), 1), Xyzti(vertcat(roadEdgeCands{ : }), 2), Xyzti(vertcat(roadEdgeCands{ : }), 3), 'square', 'color', 'y', 'markersize', 20, 'linewidth', 2)

% plot3(pc.x, pc.y, pc.z, '.', 'color', 0.7*[1 1 1]); % point cloud


%% Plot classified points

ind_fig = ind_fig + 1;
f_initFig(ind_fig, 'w')

li_pc_marking_laneSeparator = pc.classification == classLabel_marking_laneSeparator;
li_pc_bm_left = pc.classification == classLabel_marking_border_left;
li_pc_bm_right = pc.classification == classLabel_marking_border_right;

% update classification of unclassified points
li_pc_unclassif = pc.classification == classLabel_unclassified;

plot3(pc.x(li_pc_bm_left), pc.y(li_pc_bm_left), pc.z(li_pc_bm_left), '.')
plot3(pc.x(li_pc_bm_right), pc.y(li_pc_bm_right), pc.z(li_pc_bm_right), '.')
plot3(pc.x(li_pc_marking_laneSeparator), pc.y(li_pc_marking_laneSeparator), pc.z(li_pc_marking_laneSeparator), '.')
% plot3(pc.x(li_pc_laneArea), pc.y(li_pc_laneArea), pc.z(li_pc_laneArea), '.')



plot3(pc.x(li_pc_roadSurfEdge), pc.y(li_pc_roadSurfEdge), pc.z(li_pc_roadSurfEdge), '.', 'markersize', 20)

% plot3(pc.x(li_pc_unclassif), pc.y(li_pc_unclassif), pc.z(li_pc_unclassif), '.', 'markersize', 2, 'color', 0.7*[1 1 1])

%% Plot a subset of profiles 

% % % ind_prof_start = 950;
% % % ind_prof_end = 1000;
% % % 
% % % % Select some height interval picked with data cursor
% % % heightInterval = 2;
% % % li_pc_height = pc.z >= cursor_info.Position(3) - heightInterval/2 & pc.z <= cursor_info.Position(3) + heightInterval/2;
% % % 
% % % % logical indices for the points in the profile subset
% % % li_pc_profSubSet = ins_pc_prof >= ind_prof_start & ...
% % %     ins_pc_prof <= ind_prof_end & ...
% % %     li_pc_height;
% % % 
% % % % Plot subset
% % % cmap = colormap('parula');
% % % f_initFig(302, 'k')
% % % fscatter3(pc.x(li_pc_profSubSet), pc.y(li_pc_profSubSet), pc.z(li_pc_profSubSet), pc.intensity(li_pc_profSubSet), cmap);
% % % colorbar

%% plot trajectory

% % % plot3(Traj(:, ci_Traj_x), Traj(:, ci_Traj_y), Traj(:, ci_Traj_z), '.', 'markersize', 6')


%% Write las file 

% % option 'fgi_scale ...' gives preserves also millimetres
% mat2las(pc, '-fgi_scale 0.001 0.001 0.001 -o E:\Combat_roadModelling_data\RoadDataRoamerR3\VT1\160421_125641_VUX-1-HA_etrs_edges.las');

