function [h] = fscatter3_edit_Joona(X,Y,Z,C,cmap)
% [h] = fscatter3(X,Y,Z,C,cmap);
% Plots point cloud data in cmap color classes and 3 Dimensions,
% much faster and very little memory usage compared to scatter3 !
% X,Y,Z,C are vectors of the same length
% with C being used as index into colormap (can be any values though)
% cmap is optional colourmap to be used
% h are handles to the line objects

% Felix Morsdorf, Jan 2003, Remote Sensing Laboratory Zuerich
  
if nargin == 4
  numclass = 256; % Number of color classes
  cmap = jet(numclass);
elseif nargin == 5
  numclass = max(size(cmap));
  if numclass == 1
    cmap = jet(numclass);
    numclass = 256;
  end  
end

% avoid too many calculations

mins = min(C);
maxs = max(C);
minz = min(Z);
maxz = max(Z);
minx = min(X);
maxx = max(X);
miny = min(Y);
maxy = max(Y);

% construct colormap :

% cmap(224:end, 1) = 1; % ??edit by Matti: saturated colormap (without darker shades of red)

col = cmap;

% determine index into colormap

ii = round(interp1([floor(mins) ceil(maxs)],[1 numclass],C));
hold on;
%set(gcf,'color','black');
colormap(cmap);

% plot each color class in a loop

k = 0;
for j = 1:numclass
  jj = find(ii == j);
  if ~isempty(jj)
    k = k + 1;
    h(k) = plot3(X(jj),Y(jj),Z(jj),'.','color',col(j,:), ...
		 'markersize',2);
  end  
end
axis([minx maxx miny maxy minz maxz])
axis image

xlabel=('X');
ylabel=('Y');
zlabel=('Z');
%title('-WGS84-');