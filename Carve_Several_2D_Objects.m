function [II,PC] = Carve_Several_2D_Objects(xyz,nobj)
%
% 'Carve_Several_2D_Objects' lets user to select a wanted region from a point cloud.
% The function is built on MATLAB's own figure and axes objects, so all
% 2D selection and viewing options, such as pan and zoom should work.
%
% The function uses MATLAB's built-in 'inpolygon' function.
%
% SYNTAX: [IN,[PC]] = Carve_Several_2D_Objects(xyz,nobj)
%                OR
%         [IN,[PC]] = Carve_Several_2D_Objects(xyz)
% ------
% IN       = Logical point index: 1 = inside boundary, 0 = outside boundary
% PC.nobj  = chosen plane view [k x 2] as k x {1 x 2} cell arrays [optional]
% PC.Pts   = Outlining points as k x {m x 2} cell arrays          [optional]
%
% xyz      = Point cloud [n x m]
% nobj     = number of picked objects [k x 1] OR Point picking structure
%
% LMB      = On axes: Pick a new point
%            On line: Moves an already selected boundary point to the clicked position (within selection range)
%
% RMB      = On line: Delete the closest selected boundary point (within selection range)
%
% USE:
% -----
% 1. Give point cloud coordinates and number of picked objects
% 2. Select boundary region with mouse clicks
% 3. Press 'Return' key (with figure selected) to move to the next object
%    (1st and last points are connected automatically to enclose the area)
% 4. Repeat until all objects are gone through (ESC key terminates the routine and prints results)
% 5. Point selection vector prints in the workspace
%
% SAMPLE CODE:
% ------------
%
% X = rand(4000,2)*10;
% [II,PC] = Carve_Several_2D_Objects(X,10);
% 
% figure(1);
% subplot(1,2,1)
% plot(X(II > 0,1),X(II > 0,2),'b.','MarkerSize',5); axis([0 10 0 10]);
% hold on; plot(X(~II,1),X(~II,2),'r.','MarkerSize',2);
% title('Original point cloud, N = 4000')
% 
% X2 = rand(16000,2)*10;
% [II2,dummy] = Carve_Several_2D_Objects(X2,PC);
% 
% subplot(1,2,2); plot(X2(II2 > 0,1),X2(II2 > 0,2),'b.','MarkerSize',5); axis([0 10 0 10]);
% hold on; plot(X2(~II2,1),X2(~II2,2),'r.','MarkerSize',2);
% title('Dense point cloud, N = 16000')
%
%    (c) 2010, Finnish Geodetic Institute - Eetu Puttonen
%    ---------------------- 18.05.2010 -----------------------
%    ------------------ Updated  07.11.16 --------------------

%% The main code

% Initialize all points with an 'in' status
sx =  size(xyz,1);
I  = (1:sx)';
Io = zeros(sx,1);
in = false(sx,1);

if ~isstruct(nobj)
%% Manual point picking mode, needs 2 inputs
    % Create various object handles
    F = figure;

    % Give in nobj as [k x 1] vector
    s  = nobj;
    
    % Create structure to save region defining points and the used planes
    % for further use
    PC.obj = cell(s,1);
    PC.Pts = cell(s,1);
    
    if max(size(xyz)) < 10000; ms = 5; else ms = 2; end
    
    for i = 1:s
        
        % --------------------------------------------------------------------------------------------
        
        % Plot points inside area, 'HitTest' flag is set off to allow free point selection
        plot(xyz(I(in),1),xyz(I(in),2),'b.','Markersize',ms,'HitTest','Off');

        hold on
        if i == 1 % <- Whole cloud needs to be printed just once, then only the chosen points are redrawn
            if size(xyz,2) == 2
                plot(xyz(I(~in),1),xyz(I(~in),2),'r.','Markersize',ms,'HitTest','Off');
            elseif size(xyz,2) == 3
                hf = Own_qnt(xyz(:,3),0:0.1:.9); % Quantile implementation, see function in bottom
                hi = zeros(size(xyz(:,3),1),1);
                for ih = 1:10;
                    hi = hi + (xyz(:,3) >= hf(ih));
                end
                
                % Colourmap
                cm = copper(10);

                for jh = 1:10;
                    plot(xyz((hi == jh) & ~in,1),xyz((hi == jh) & ~in,2),'.', ...
                        'MarkerSize',ms,'MarkerFaceColor',cm(jh,:),'MarkerEdgeColor',cm(jh,:),'HitTest','Off');
                    axis('equal');
                end;

                colorbar;
                colormap(copper);
                set(gca,'Clim',[hf(2) hf(end-1)])

            end
        end
        
        if i == 1
            axis('equal')
            a = axis;
            axis(a + 0.05*[-(a(2)-a(1)) (a(2)-a(1)) -(a(4)-a(3)) (a(4)-a(3))]);            
        else
            axis(a)
        end
        
        % --------------------------------------------------------------------------------------------
        
        A = gca;
        
        L = line;
        
        % Initialize figure handle
        set(F, ...
            'KeyReleaseFcn',{@FigKeyRel,F,L}, ...
            'UserData',[]);
        
        % Initialize axes handle
        set(A, ...
            'FontSize',14, ...
            'ButtonDownFcn',{@BDown,A,L}, ...
            'UserData',[]);
        set(get(A,'XLabel'),'String','x','FontSize',16)
        set(get(A,'YLabel'),'String','y','FontSize',16)
        
        % Initialize line object handle
        set(L, ...
            'Color',[1 0 0], ...
            'ButtonDownFcn', {@BDown_Line,A,L}, ...
            'XData', [], ...
            'YData', [], ...
            'Marker', '*');

        waitfor(F,'UserData',1)
        Fh = get(L,'UserData'); % <- The actual button press information is saved in the line object 'user data'
        
        if ~isempty(Fh)
            % Wait for 'return' key to be pressed and released to continue
            
            if ~isempty(get(A,'UserData')) && size(get(A,'UserData'),1) >= 3
                XY = get(A,'UserData');
                % Pick points inside the chosen area
                [IN, ON] = inpolygon(xyz(:,1),xyz(:,2),XY(:,1),XY(:,2)); % MATLAB inpolygon
                in(IN | ON) = true;
                Io(in & (Io == 0)) = i;
                
                PC.obj{i} = i;
                PC.Pts{i} = XY;
                
                set(A,'UserData',[])
                set(L,'XData', [],'YData', [],'UserData',[])
            end
            
            if Fh == 0
                % Wait for 'escape' key to be pressed and released to stop point picking
                II = Io;
                close(F);
                
                % Remove empty cells if the search is canceled
                Ii = cellfun('isempty',PC.obj);
                Ij = cellfun('isempty',PC.Pts);
                
                % Keep filled cells
                PC.obj = PC.obj(~Ii & ~Ij); % <- Remove 'objects' with no hits in them
                PC.Pts = PC.Pts(~Ij & ~Ii);
                
                return                
            end
        end
        
        % close image after all objects have been picked
        if i == s
            % Remove possible empty cells
            eC = cellfun(@isempty,PC.obj);
            PC.obj(eC) = [];
            PC.Pts(eC) = [];            
            close(F)
        end
        II = Io;
    end
elseif isstruct(nobj)
%% Automatic point picking, needs 'PC' type structure as an input    
    so  = size(nobj.obj,1);
    for i = 1:so
        % Pick points inside the chosen area
            [IN, ON] = inpolygon(xyz(:,1),xyz(:,2),nobj.Pts{i}(:,1),nobj.Pts{i}(:,2)); % MATLAB inpolygon
            in(IN | ON) = true;       
            Io(in & (Io == 0)) = i;
    end
    II = Io;
    PC = nobj; % Point picking area doesn't change in this case
end

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
function FigKeyRel(src,eventdata,F,L)
% Set up a kill flag for the figure
if strcmpi(get(F,'CurrentCharacter'),char(13))
    set(F,'UserData',1)
    set(L,'UserData',1) % <- Use the line object to carry the actual button press information on as figure value is used on 'waitfor'
elseif strcmpi(get(F,'CurrentCharacter'),char(27))
    set(F,'UserData',1)
    set(L,'UserData',0) % <- Use the line object to carry the actual button press information on as figure value is used on 'waitfor'
end
end

% Own 'quantile' function (to avoid using statistics toolbox)
function Out = Own_qnt(X,vals)
% Reference in:
% http://stackoverflow.com/questions/2384977/finding-99-coverage-in-matlab
% Also, check matlabs commands 'quantile' and 'prctile' in Statistics Toolbox if available

% Quick check for inputs
if ~issorted(vals); error('Quantile range needs to be sorted.'); end
if any(vals < 0) || any(vals > 1); error('Quantile range '); end

% Check that value vector is a column
if size(vals,2) > size(vals,1); vals = vals'; end

% Sort values
Xs = sort(X(:));
N = size(Xs,1);

% Get quantile indices
Out = interp1q([0 (0.5:(N-0.5))./N 1]',Xs([1 1:N N],:),vals);

end