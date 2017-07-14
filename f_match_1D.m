function ind_match = f_match_1D(q, x)
%
% ind_match = f_match_1D(q, x)
%
% Find a best match for a scalar query point q from a set x (vector) using
% 1D Euclidean distance.
%
%
% ------
% Input:
%
%   q   A query point (scalar)
%   x   (n x 1) A set from which a best match is searched for.
%
% -------
% Output:
%
%   ind_match   Index of the element of x that is closest to q.
%
%
%
% -------------------------------------------------
% Current version 30.1.2016, (c) M.Lehtomäki, 2016
%
% Remarks:
%   -   Help is checked and ok
%   -   Checked line by line and tested in practice and should be ok
%       (31012016)
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

dist = abs( f_repmat_mtx_fast( q, length( x ), 1 ) - x ); % absolute differences rivi tsekattu ja ok
[~, ind_match] = min( dist ); % the best match rivi tsekattu ja ok

end
