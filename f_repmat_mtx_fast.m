function B = f_repmat_mtx_fast(A, M, N)
%
% B = f_repmat_mtx_fast(A, M, N)
% 
% A faster version of repmat for matrices. Does not iclude checks of the
% input. Adapted from repmat.
%
%
% Input:
%
%   A       The original matrix or vector to be replicated.
%   M       The number of times A is replicated vertically.
%   N       The number of times A is replicated horizontally.
%
% Output:
%   B       Replicated A.
%
% Calls functions (children):
%   no children (31.10.2011)
%
% Version 20.1.2011 by Matti Lehtomäki
%   Remarks:    -
%


if nargin == 2
    if isscalar(M)
        siz = [M M];
    else
        siz = M;
    end
else
    siz = [M N];
end


[m,n] = size(A);

if (m == 1 && siz(2) == 1)
    B = A(ones(siz(1), 1), :);
elseif (n == 1 && siz(1) == 1)
    B = A(:, ones(siz(2), 1));
else
    mind = (1:m)';
    nind = (1:n)';
    mind = mind(:,ones(1,siz(1)));
    nind = nind(:,ones(1,siz(2)));
    B = A(mind,nind);
end


end
