function f_initFig(indFig, col)
% 
% f_initFig(indFig, col)
% 
% Initialise figure
% 
% Input:
%   indFig  Index of the figure
%   col     Background color of the axis (default white)
% 

figure(indFig), close, figure(indFig)
set(gca, 'dataaspectratio', [1 1 1]);
hold on;
plotbrowser

if nargin == 2
    set(gca, 'color', col)
end

end
