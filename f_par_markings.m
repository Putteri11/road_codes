function par = f_par_markings(roadClass)
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
par = rmfield(par, 'thre_heightJump'); % do not perform height jump extraction
par = rmfield(par, 'ul_dist_cc'); % do not perform scan line segmentation

end

