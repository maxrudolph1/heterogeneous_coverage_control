% Coverage control using Lloyd's Algorithm.

% Max Rudolph
% 9/2019

%% Experiment Constants
close all;
%Run the simulation for a specific number of iterations
iterations = 2000;

%% Set up the Robotarium object

N = 2;
K = 2;
x_init = generate_initial_conditions(N+K); %,'Width',1.1,'Height',1.1,'Spacing', 0.35);
x_init = x_init - [min(x_init(1,:)) - (-1.6 + 0.2);min(x_init(2,:)) - (-1 + 0.2);0];

r = Robotarium('NumberOfRobots', N+K, 'ShowFigure', true,'InitialConditions',x_init);


%Initialize velocity vector
dxir = zeros(2, N);
dxid = zeros(2, K);

%Boundary
crs = [r.boundaries(1), r.boundaries(3);
    r.boundaries(1), r.boundaries(4);
    r.boundaries(2), r.boundaries(4);
    r.boundaries(2), r.boundaries(3)];

% crs = [r.boundaries(1), r.boundaries(3);
%        r.boundaries(1), r.boundaries(4);
%        1/3*r.boundaries(1), r.boundaries(4);
%        1/3*r.boundaries(1), 0;
%        1/3*r.boundaries(2), 0;
%        1/3*r.boundaries(2), r.boundaries(4);
%        r.boundaries(2), r.boundaries(4);
%        r.boundaries(2), r.boundaries(3)];

%Gausian Setup
center = [0;0];
sigma = .2*eye(2);
detSigma = det(sigma);
%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics();
% Single-integrator barrier certificates
uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary();
si_barrier_cert_boundary = create_si_barrier_certificate();
% Single-integrator position controller
motion_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 1);

%% Plotting Setup
marker_size = determine_marker_size(r, 0.08);

poses = r.get_poses();
x = poses(:,1:N);
xd = poses(:,N+1:end);

verCellHandle = zeros(N,1);
verCellHandleD = zeros(K,1);
cellColors = cool(N);
cellColorsD = cool(K);

for i = 1:N % color according to robot
    verCellHandle(i)  = patch(x(1,i),x(2,i),cellColors(i,:),'FaceAlpha', 0.3); % use color i  -- no robot assigned yet
    hold on
end

pathHandle = zeros(N,1);
pathHandleD = zeros(K,1);
for i = 1:N % color according to
    pathHandle(i)  = plot(x(1,i),x(2,i),'-.','color',cellColors(i,:)*.9, 'LineWidth',4);
end
centroidHandle = plot(x(1,:),x(2,:),'+','MarkerSize',marker_size, 'LineWidth',2, 'Color', 'k');



for i = 1:N % color according to
    xD = [get(pathHandle(i),'XData'),x(1,i)];
    yD = [get(pathHandle(i),'YData'),x(2,i)];
    set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
end


plotRes = 100;
xArray = linspace(r.boundaries(1),r.boundaries(2),plotRes);
yArray = linspace(r.boundaries(3),r.boundaries(4),plotRes);
zArray = zeros(plotRes);

for i = 1:length(xArray)
    for j = 1:length(yArray)
        zArray(i,j) = gaussC(xArray(i),yArray(j),sigma, detSigma, center);
    end
end
potentialHandle = contour(xArray,yArray,zArray', 'LineWidth',1, 'LevelStep',0.1,'ShowText','on');



for i = 1:K % color according to drone
    verCellHandleD(i)  = patch(xd(1,i),xd(2,i),cellColorsD(i,:),'FaceAlpha', 0.01, 'LineWidth', 4); % use color i  -- no robot assigned yet
    %verCellHandleD(i)  = patch(xd(1,i),xd(2,i),'EdgeColor','green','FaceColor','none','LineWidth',2);
    hold on
end

for i = 1:K % color according to
    pathHandleD(i) = plot(xd(1,i),xd(2,i),'-.','color',cellColorsD(i,:)*.9, 'LineWidth',4);
end
%centroidHandleD = plot(xd(1,:),xd(2,:),'+','MarkerSize',marker_size, 'LineWidth',2, 'Color', 'k');


for i = 1:K % color according to
    xDd = [get(pathHandleD(i),'XData'),xd(1,i)];
    yDd = [get(pathHandleD(i),'YData'),xd(2,i)];
    set(pathHandleD(i),'XData',xDd,'YData',yDd);%plot path position
end

xArrayD = linspace(r.boundaries(1),r.boundaries(2),plotRes);
yArrayD = linspace(r.boundaries(3),r.boundaries(4),plotRes);
zArrayD = ones(plotRes);

r.step();

%% Main Loop
in_pos = false;
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    poses = r.get_poses();
    x = poses(:, 1:N);
    xd = poses(:, N+1:end);
    
    % Convert to SI states
    xi = uni_to_si_states(x);
    xid = uni_to_si_states(x);
    
    %% Algorithm
    
    [Px, Py] = lloydsAlgorithmWeighted(x(1,:)',x(2,:)',xd(1,:)',xd(2,:)', crs, verCellHandle, 100, center, sigma, detSigma, true);
    dxir = motion_controller(x(1:2, :), [Px';Py']);
   
    [Pxd, Pyd] = lloydsAlgorithm(xd(1,:)',xd(2,:)', crs, verCellHandleD, true);
    dxid = motion_controller(xd(1:2, :), [Pxd';Pyd']);
    
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxir(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxir(:, to_thresh) = threshold*dxir(:, to_thresh)./norms(to_thresh);
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(xd) norm(dxid(:, xd)), 1:K);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxid(:, to_thresh) = threshold*dxid(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxir, x);
    dxu = uni_barrier_cert_boundary(dxu, x);
    
    dxud = si_to_uni_dyn(dxid, xd);
    dxud = uni_barrier_cert_boundary(dxud, xd);
    
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:(N+K), [dxu*in_pos dxud]);
    
    
    
    %% Update Plot Handles
    
    for i = 1:N % color according to
        xD = [get(pathHandle(i),'XData'),x(1,i)];
        yD = [get(pathHandle(i),'YData'),x(2,i)];
        set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
    end
    
    set(centroidHandle,'XData',Px,'YData',Py);%plot centroid position
    
    %for i = 1:K % color according to
    %   xDd = [get(pathHandleD(i),'XData'),xd(1,i)];
    %   yDd = [get(pathHandleD(i),'YData'),xd(2,i)];
    %   set(pathHandleD(i),'XData',xDd,'YData',yDd);%plot path position
    %end
    
    %set(centroidHandleD,'XData',Pxd,'YData',Pyd);%plot centroid position
    
    %Iterate experiment
    r.step();
    if t == 1
        in_pos = true;
    end
    
    
    
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

%% Helper Functions

% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end


%% Lloyds algorithm put together by Aaron Becker

function [Px, Py] = lloydsAlgorithmWeighted(Px,Py,Pxd, Pyd, crs, verCellHandle, res, center, sigma, detSigma, plt)
% LLOYDSALGORITHM runs Lloyd's algorithm on the particles at xy positions
% (Px,Py) within the boundary polygon crs for numIterations iterations
% showPlot = true will display the results graphically.
%
% Lloyd's algorithm starts with an initial distribution of samples or
% points and consists of repeatedly executing one relaxation step:
%   1.  The Voronoi diagram of all the points is computed.
%   2.  Each cell of the Voronoi diagram is integrated and the centroid is computed.
%   3.  Each point is then moved to the centroid of its Voronoi cell.
%
% Inspired by http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
% Requires the Polybool function of the mapping toolbox to run.
%
% Run with no input to see example.  To initialize a square with 50 robots
% in left middle, run:
%lloydsAlgorithm(0.01*rand(50,1),zeros(50,1)+1/2, [0,0;0,1;1,1;1,0], 200, true)
%
% Made by: Aaron Becker, atbecker@uh.edu
format compact

% initialize random generator in repeatable fashion
sd = 20;
rng(sd)

%         crs = [ 0, 0;
%             0, yrange;
%             1/3*xrange, yrange;  % a world with a narrow passage
%             1/3*xrange, 1/4*yrange;
%             2/3*xrange, 1/4*yrange;
%             2/3*xrange, yrange;
%             xrange, yrange;
%             xrange, 0];

xrange = max(crs(:,1)) - min(crs(:,1));
yrange = max(crs(:,2)) - min(crs(:,2));

% Apply LLYOD's Algorithm
[v,c]=VoronoiBounded(Px,Py,crs);

[vd cd] = VoronoiBounded(Pxd, Pyd, crs);




for i = 1:numel(c) %calculate the center of mass of each cell


    xVoronoi = linspace(min(v(c{i},1)),max(v(c{i},1)),res);  %linspace(min(v(c{i},1)),max(v(c{i},1)),res);
    yVoronoi = linspace(min(v(c{i},2)),max(v(c{i},2)),res);  %linspace(min(v(c{i},2)),max(v(c{i},2)),res);
    
    whichDrone = 0;
    for l = 1:numel(cd)
        if inpolygon(Px(i), Py(i), vd(cd{l}, 1), vd(cd{l}, 2))
            whichDrone = l;
        end
        
    end
    
    if whichDrone == 0
        Px(i) = Px(i) + .01;
        Py(i) = Py(i) + .01;
        for l = 1:numel(cd)
            if inpolygon(Px(i), Py(i), vd(cd{l}, 1), vd(cd{l}, 2))
                whichDrone = l;
                
            end
            
        end
    end
    coords = [];
    
    for a = 1:numel(yVoronoi)
        coords = [coords; [xVoronoi' yVoronoi(a)*ones(numel(xVoronoi),1)]];
    end
    
   
    in1 = inpolygon(coords(:,1),coords(:,2),v(c{i},1),v(c{i},2));
    in2 = inpolygon(coords(:,1),coords(:,2),vd(cd{whichDrone},1),vd(cd{whichDrone},2));
   
    
    xArrayIn = coords(in1 & in2, 1)';
    yArrayIn = coords(in1 & in2, 2)';
    positionMassSumX = 0;
    positionMassSumY = 0;
    totalMass = 0;

    for j = 1:length(xArrayIn)
            zArrayIn = gaussC(xArrayIn(j),yArrayIn(j), sigma, detSigma, center);
            positionMassSumX = positionMassSumX + zArrayIn*xArrayIn(j);
            positionMassSumY = positionMassSumY + zArrayIn*yArrayIn(j);
            totalMass = totalMass + zArrayIn;
    end
    
    
    positionMassSumX = positionMassSumX/(totalMass);
    positionMassSumY = positionMassSumY/(totalMass);
    cx = positionMassSumX;
    cy = positionMassSumY;
    cx = min(max(crs(:,1)),max(min(crs(:,1)), cx));
    cy = min(max(crs(:,2)),max(min(crs(:,2)), cy));
    if ~isnan(cx) && inpolygon(cx,cy,crs(:,1),crs(:,2))
        Px(i) = cx;  %don't update if goal is outside the area
        Py(i) = cy;
    end
end
if plt
    for i = 1:numel(c) % update Voronoi cells
        set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
    end
end
end

function [Cx,Cy] = PolyCentroid(X,Y)
% POLYCENTROID returns the coordinates for the centroid of polygon with vertices X,Y
% The centroid of a non-self-intersecting closed polygon defined by n vertices (x0,y0), (x1,y1), ..., (xn?1,yn?1) is the point (Cx, Cy), where
% In these formulas, the vertices are assumed to be numbered in order of their occurrence along the polygon's perimeter, and the vertex ( xn, yn ) is assumed to be the same as ( x0, y0 ). Note that if the points are numbered in clockwise order the area A, computed as above, will have a negative sign; but the centroid coordinates will be correct even in this case.http://en.wikipedia.org/wiki/Centroid
% A = polyarea(X,Y)

Xa = [X(2:end);X(1)];
Ya = [Y(2:end);Y(1)];

A = 1/2*sum(X.*Ya-Xa.*Y); %signed area of the polygon

Cx = (1/(6*A)*sum((X + Xa).*(X.*Ya-Xa.*Y)));
Cy = (1/(6*A)*sum((Y + Ya).*(X.*Ya-Xa.*Y)));
end

function [V,C]=VoronoiBounded(x,y, crs)
% VORONOIBOUNDED computes the Voronoi cells about the points (x,y) inside
% the bounding box (a polygon) crs.  If crs is not supplied, an
% axis-aligned box containing (x,y) is used.

bnd=[min(x) max(x) min(y) max(y)]; %data bounds
if nargin < 3
    crs=double([bnd(1) bnd(4);bnd(2) bnd(4);bnd(2) bnd(3);bnd(1) bnd(3);bnd(1) bnd(4)]);
end

rgx = max(crs(:,1))-min(crs(:,1));
rgy = max(crs(:,2))-min(crs(:,2));
rg = max(rgx,rgy);
midx = (max(crs(:,1))+min(crs(:,1)))/2;
midy = (max(crs(:,2))+min(crs(:,2)))/2;

% add 4 additional edges
xA = [x; midx + [0;0;-5*rg;+5*rg]];
yA = [y; midy + [-5*rg;+5*rg;0;0]];

[vi,ci]=voronoin([xA,yA]);

% remove the last 4 cells
C = ci(1:end-4);
V = vi;
% use Polybool to crop the cells
%Polybool for restriction of polygons to domain.

for ij=1:length(C)
    % thanks to http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
    % first convert the contour coordinate to clockwise order:
    [X2, Y2] = poly2cw_custom(V(C{ij},1),V(C{ij},2));
    tempA = polyshape(crs(:,1), crs(:,2),'Simplify',false);
    tempB = polyshape(X2, Y2,'Simplify',false);
    tempC = intersect(tempA,tempB);
    [xb, yb] = boundary(tempC);
    %[xb, yb] = polybool('intersection',crs(:,1),crs(:,2),X2,Y2);
    ix=nan(1,length(xb));
    for il=1:length(xb)
        if any(V(:,1)==xb(il)) && any(V(:,2)==yb(il))
            ix1=find(V(:,1)==xb(il));
            ix2=find(V(:,2)==yb(il));
            for ib=1:length(ix1)
                if any(ix1(ib)==ix2)
                    ix(il)=ix1(ib);
                end
            end
            if isnan(ix(il))==1
                lv=length(V);
                V(lv+1,1)=xb(il);
                V(lv+1,2)=yb(il);
                ix(il)=lv+1;
            end
        else
            lv=length(V);
            V(lv+1,1)=xb(il);
            V(lv+1,2)=yb(il);
            ix(il)=lv+1;
        end
    end
    C{ij}=ix;
    
end
end

function [Px, Py] = lloydsAlgorithm(Px,Py, crs, verCellHandle, plt)
% LLOYDSALGORITHM runs Lloyd's algorithm on the particles at xy positions
% (Px,Py) within the boundary polygon crs for numIterations iterations
% showPlot = true will display the results graphically.
%
% Lloyd's algorithm starts with an initial distribution of samples or
% points and consists of repeatedly executing one relaxation step:
%   1.  The Voronoi diagram of all the points is computed.
%   2.  Each cell of the Voronoi diagram is integrated and the centroid is computed.
%   3.  Each point is then moved to the centroid of its Voronoi cell.
%
% Inspired by http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
% Requires the Polybool function of the mapping toolbox to run.
%
% Run with no input to see example.  To initialize a square with 50 robots
% in left middle, run:
%lloydsAlgorithm(0.01*rand(50,1),zeros(50,1)+1/2, [0,0;0,1;1,1;1,0], 200, true)
%
% Made by: Aaron Becker, atbecker@uh.edu
format compact

% initialize random generator in repeatable fashion
sd = 20;
rng(sd)

%         crs = [ 0, 0;
%             0, yrange;
%             1/3*xrange, yrange;  % a world with a narrow passage
%             1/3*xrange, 1/4*yrange;
%             2/3*xrange, 1/4*yrange;
%             2/3*xrange, yrange;
%             xrange, yrange;
%             xrange, 0];

xrange = max(crs(:,1)) - min(crs(:,1));
yrange = max(crs(:,2)) - min(crs(:,2));

% Apply LLYOD's Algorithm
[v,c]=VoronoiBounded(Px,Py,crs);

for i = 1:numel(c) %calculate the centroid of each cell
    [cx,cy] = PolyCentroid(v(c{i},1),v(c{i},2));
    cx = min(max(crs(:,1)),max(min(crs(:,1)), cx));
    cy = min(max(crs(:,2)),max(min(crs(:,2)), cy));
    if ~isnan(cx) && inpolygon(cx,cy,crs(:,1),crs(:,2))
        Px(i) = cx;  %don't update if goal is outside the polygon
        Py(i) = cy;
    end
end
if plt
    for i = 1:numel(c) % update Voronoi cells
        set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
    end
end
end


function [ordered_x, ordered_y] = poly2cw_custom(x,y)
cx = mean(x);
cy = mean(y);
a = atan2(y-cy, x -cx);

[~, order] = sort(a);
ordered_x = x(order);
ordered_y = y(order);
end

function val = gaussC(x, y, sigma, detSigma, center)
xc = center(1);
yc = center(2);
exponent = ((x-xc).^2/sigma(1,1) + (y-yc).^2/sigma(2,2))./(2);
amplitude = 1 / (sqrt(detSigma) * 2*pi);
val = amplitude  * exp(-exponent);
end