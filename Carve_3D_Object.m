function [II,PC] = Carve_3D_Object(xyz,order)
%
% 'Carve_Object' lets user to select a wanted region from a point cloud.
% The function is built on MATLAB's own figure and axes objects, so all
% 2D selection and viewing options, such as pan and zoom should work.
%
% The function uses MATLAB's built-in 'inpolygon' function.
%
% SYNTAX: [IN,[PC]] = Carve_3D_Object(xyz,order)
%                OR
%         [IN,[PC]] = Carve_3D_Object(xyz)
% ------
% IN       = Logical point index: 1 = inside boundary, 0 = outside boundary
% PC.Order = chosen view  direction [k x 2] as k x {1 x 2} cell arrays [optional]
% PC.Pts   = Outlining points as k x {m x 2} cell arrays               [optional]
%
% xyz      = Point cloud [n x m]
% order    = chosen view direction (MATLAB's view)[k x 2] OR Point picking structure (PC)
%            XZ plane = [0  0]
%            XY plane = [0 90]
%            YZ plane = [90 0]
%
% LMB      = On axes: Pick a new point
%            On line: Moves an already selected boundary point to the clicked position (within selection range)
%
% RMB      = On line: Delete the closest selected boundary point (within selection range)
%
% USE:
% -----
% 1. Give point cloud coordinates and view order
% 2. Select boundary region with mouse clicks
% 3. Press 'Return' key (with figure selected) to move to the next view plane
%    (1st and last points are connected automatically to enclose the area)
% 4. Repeat until all planes are gone through
% 5. Point status vector (and an optional selection structure) print into workspace
%
% SAMPLE CODE:
% ------------
%
% X = rand(4000,3)*10;
% [II,PC] = Carve_3D_Object(X,[-37.5 30; 0 90; 0 0; 37.5 37.5; 90 0]);
% 
% figure; 
% subplot(1,2,1); plot3(X(II,1),X(II,2),X(II,3),'b.','MarkerSize',5); axis('equal'); axis([0 10 0 10 0 10]);
% hold on; plot3(X(~II,1),X(~II,2),X(~II,3),'r.','MarkerSize',2); box on
% title('Original point cloud, N = 4000')
% 
% X2 = rand(16000,3)*10;
% [II2,dummy] = Carve_3D_Object(X2,PC);
% 
% subplot(1,2,2); plot3(X2(II2,1),X2(II2,2),X2(II2,3),'b.','MarkerSize',5); axis('equal'); axis([0 10 0 10 0 10]);
% hold on; plot3(X2(~II2,1),X2(~II2,2),X2(~II2,3),'r.','MarkerSize',2); box on
% title('Densified point cloud, N = 16000')
% 
%  (c) 2010, Finnish Geodetic Institute - Eetu Puttonen
%  ----------------------- 18.05.2010 ------------------------
%  ------------------ Updated  07.11.2016 --------------------

%% The main code

% Initialize all points with an 'in' status
sx =  size(xyz,1);
I  = (1:sx)';
in = true(sx,1);

if ~isstruct(order)
    %% Manual point picking mode, needs 2 inputs
    % Give in order as [n x 2] or [n x 3] matrix
    s  = size(order,1);
    
    % Create structure to save region defining points and the used planes
    % for further use
    PC.Order = cell(s,1);
    PC.Pts   = cell(s,1);
    
    for i = 1:s
        
        % Do 2D projection of the point cloud
        if size(order,2) == 2
            M = viewmtx(order(i,1),order(i,2));
        elseif size(order,2) == 3
            M = viewmtx(order(i,1),order(i,2),order(i,3));
        end
        C2D = M*([xyz,ones(size(xyz,1),1)])';
        C2D = C2D(1:2,:)';
        
        if size(C2D,1) >= 10000; m = 2;
        else m = 5;
        end
        
        % Create various object handles
        F = figure;
        
        if ~all(in)
            hold on
            plot(C2D(I(~in),1),C2D(I(~in),2),'c.','MarkerSize',m,'HitTest','Off');
        end
        
        % Plot points inside area, 'HitTest' flag is set off to allow free point selection
        plot(C2D(I(in),1),C2D(I(in),2),'b.','MarkerSize',m,'HitTest','Off');
        
        axis('equal')
        a = axis;
        axis(a + 0.05*[-(a(2)-a(1)) (a(2)-a(1)) -(a(4)-a(3)) (a(4)-a(3))]);
        
        A = gca;
        
        L = line;
        
        % Initialize figure handle
        set(F, ...
            'KeyReleaseFcn',{@FigKeyRel,F});
        
        % Initialize axes handle
        set(A, ...
            'FontSize',14, ...
            'ButtonDownFcn',{@BDown,A,L}, ...
            'UserData',[]);
        set(get(A,'XLabel'),'String',num2str(order(i,1)),'FontSize',16)
        set(get(A,'YLabel'),'String',num2str(order(i,2)),'FontSize',16)
        
        if size(order,2) == 2
            title(['Azimuth: ',num2str(order(i,1)), ', Elevation: ',num2str(order(i,2))],'FontSize',20);
        elseif size(order,2) == 3
            title(['Azimuth: ',num2str(order(i,1)), ', Elevation: ',num2str(order(i,2)), ', Phi: ',num2str(order(i,3))],'FontSize',20);
        end
        
        % Initialize line object handle
        set(L, ...
            'Color',[1 0 0], ...
            'ButtonDownFcn', {@BDown_Line,A,L}, ...
            'XData', [], ...
            'YData', [], ...
            'Marker', '*');
        
        % Wait for 'return' key to be pressed and released to continue
        waitfor(F,'UserData',1)
        if ~isempty(get(A,'UserData')) && size(get(A,'UserData'),1) >= 3
            XY = get(A,'UserData');
            % Pick points inside the chosen area
            [IN, ON] = inpolygon(C2D(:,1),C2D(:,2),XY(:,1),XY(:,2)); % MATLAB inpolygon
            in(~IN & ~ON) = false;
            
            if size(order,2) == 2
                PC.Order{i} = [order(i,1) order(i,2)];
            elseif size(order,2) == 3
                PC.Order{i} = [order(i,1) order(i,2) order(i,3)];
            end
            PC.Pts{i} = XY; % NOTE: These are the xy coordinates of the PROJECTED PLANE!
        end
        % close temporary image
        close(F)
        II = in;
    end
elseif isstruct(order)
    %% Automatic point picking, needs 'PC' type structure as an input
    so  = size(order.Order,1);
    for i = 1:so
        
        % Do 2D projection of the point cloud
        if size(order.Order{i},2) == 2
            M = viewmtx(order.Order{i}(1,1),order.Order{i}(1,2));
        elseif size(order.Order{i},2) == 3
            M = viewmtx(order.Order{i}(1,1),order.Order{i}(1,2),order.Order{i}(1,3));
        end
        C2D = M*([xyz,ones(size(xyz,1),1)])';
        C2D = C2D(1:2,:)';
        
        % Pick points inside the chosen area
        [IN, ON] = inpolygon(C2D(:,1),C2D(:,2),order.Pts{i}(:,1),order.Pts{i}(:,2)); % MATLAB inpolygon
        in(~IN & ~ON) = false;
    end
    II = in;
    PC = order; % Point picking area doesn't change in this case
end

% Remove empty cells if the search is canceled
Ii = cellfun('isempty',PC.Order);
Ij = cellfun('isempty',PC.Pts);

% Keep filled cells
PC.Order = PC.Order(~Ii & ~Ij); % <- Remove 'objects' with no hits in them
PC.Pts   = PC.Pts(~Ij & ~Ii);

end

% ButtonDownFcn for the axes
function BDown(src,eventdata,A,L)

point = get(A,'CurrentPoint');
point = point(1,1:2);
button = get(gcf,'SelectionType');

if strcmpi(button,'normal')
    % Update userdata
    set(A, ...
        'UserData',[get(A,'UserData');point]);
    XY = get(A,'UserData');
    
    % Update line object
    set(L, ...
        'XData', XY(:,1)', ...
        'YData', XY(:,2)');
end

end

% ButtonDownFcn for the line
function BDown_Line(src,eventdata,A,L)

dist = 0.5;

button = get(gcf,'SelectionType');

X = get(L,'Xdata');
Y = get(L,'YData');

p = get(A,'CurrentPoint');
p = p(1,1:2);

[D, I] = min((X-p(1)).^2 + (Y-p(2)).^2,[],2);

if strcmpi(button,'normal')
    % Change location of the closest selected point
    if D < dist^2
        X(I) = p(1);
        Y(I) = p(2);
    end
else
    % Remove the closest point
    if D < dist^2
        X(I) = [];
        Y(I) = [];
    end
end

set(A,'UserData',[X',Y']);
set(L,'XData',X,'YData',Y);

end

% Key released function for the figure
function FigKeyRel(src,eventdata,F)
% Set up a kill flag for the figure
if strcmpi(get(F,'CurrentCharacter'),char(13))
    set(F,'UserData',1)
end
end
