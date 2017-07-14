function par = f_par_roadModelling(roadClass)
%
% [outputVar_1, outputVar_2] = f_functionName(inputVar_1, inputVar_2)
%
% Parameters of road modelling.
%
%
% ------
% Input:
%
%   roadClass (string) Road class: motor way etc.
%
% -------
% Output:
%
%   par             Struct that contains parameter values as fields
%
%
%
% -------------------------------------------------
% Current version 14.3.2017, (c) M.Lehtomäki, 2017
%
% Remarks:
%   -
%
%
% -------------------
% Previous versions:
%
% -------------------------------------------------
% Version dd.mm.yyyy, (c) M.Lehtomäki, yyyy
%
% Updates to previous versions:
%   -
%
% Remarks:
%   -
%
%
%

%% VT1 motari

% use these as default values. make changes for other roads below

% Pre-processing (noise filtering etc.)
par.thre_int = 800; % intensity threshold

% upper limit for the profile spacing
par.ul_profSpacing = 0.2; % (metres)

% Scan line segmentation
par.ul_dist_cc = 0.2; % ??function of f_mirror (metres) Upper limit for the distance in connected component (CC) labelling
par.n_neigh_cc = 30; % (number of points) the size of the neighbourhood that is checked in CC labelling
par.thre_n_pts_cc = 50; % ??function of f_mirror (number of points) threshold for the size of a CC that is preserved (smaller are discarded)
par.thre_divideLineSegment = 0.025; % (metres) Threshold for dividing a line segment into two line segments

% Road edge candidate extraction
par.thre_heightJump = 0.01; % threshold for a height jump along the profile
par.windowSize_heightJump = 10; % Window size in height jump extraction
par.thre_intensityJump = 300; % threshold for an intensity jump along the profile

% Road edge extraction
par.width_verge_shoulder = 4; % (metres) approximate width of the shoulders and verge
par.rn_pointAttr = 0.3; % (metres) radius of the neighbourhood in point attribute calculation (flatness etc.)
par.thre_flatness_roadSurf = 0.99; % (in the range [0.67, 1]) threshold for the flatness value of a point neighbourhood such that it is classified as flat

par.thre_linearity = 0.9; % (in the range [0.33, 1]) threshold for the linearity value of a point neighbourhood such that it is classified as linear
par.ul_alphaVert_hor = pi/4; % (radians) upper limit for the angle between the local plane normal and vertical direction such that the point is classified as horizontal

% Road painting extraction
par.n_neigh_cc_intJumps = 5;  % (number of points) the size of the neighbourhood that is checked in CC labelling
par.thre_length_longMarking = 50; % (metres) minimum length of a long road marking

% Curve extraction
par.r_searchCand_markings = 0.2; % (metres) search radius for a candidate from an adjacent profile (road marking extr)
par.thre_l_curve_markings = 0; % (metres) Minimum length of a curve in marking extr
ul_metres_jumpOver = 2; % (metres) over how many metres can we jump when growing the curve
par.ul_n_profs_jumpOver = round( ul_metres_jumpOver / par.ul_profSpacing ); % over how many profiles can we jump when growing the curve
par.thre_length_marking_border = 5; % (metres) minimum length of a border marking
par.thre_length_marking_centre = 1; % (metres) minimum length of a centre marking

%% Nikkilän maantie

if strcmp(roadClass, 'Nikkila') 
    
    
    par.ul_profSpacing = 0.1; % (metres) upper limit for the profile spacing
    ul_metres_jumpOver = 0.5; % (metres) over how many metres can we jump when growing the curve
    par.ul_n_profs_jumpOver = round( ul_metres_jumpOver / par.ul_profSpacing ); % over how many profiles can we jump when growing the curve
    par.r_searchCand_markings = 0.1; % (metres) search radius for a candidate from an adjacent profile (road marking extr)
    
    
    
end

end

