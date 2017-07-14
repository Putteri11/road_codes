function par = f_par_rse(roadClass)
%
% [outputVar_1, outputVar_2] = f_functionName(inputVar_1, inputVar_2)
%
% Parameters of road surface edge extraction.
%
%
% ------
% Input:
%
%   roadClass   (string) Road class: motor way etc. ??under construction;
%               now using only road names.
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

par = f_par_roadModelling(roadClass); % default values
par = rmfield(par, 'thre_intensityJump'); % do not perform intensity jump extraction
par = rmfield(par, 'ul_dist_cc'); % do not perform scan line segmentation

%% VT1 motari

if strcmp(roadClass, 'VT1')
    
    par.r_searchCand = 0.1; % (metres) search radius for a candidate from an adjacent profile (road edge surface)
    par.thre_l_curve = 2; % (metres) minimum length of a road surface edge curve
    
end

%% Nikkilän maantie

if strcmp(roadClass, 'Nikkila')
    
    par.r_searchCand = 0.1; % (metres) search radius for a candidate from an adjacent profile (road edge surface)
    par.thre_l_curve = 2; % (metres) minimum length of a road surface edge curve
    
end

end

