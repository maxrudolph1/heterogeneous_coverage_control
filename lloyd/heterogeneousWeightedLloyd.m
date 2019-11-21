% Coverage control using Lloyd's Algorithm.

% Max Rudolph
% 9/2019

%% Experiment Constants
%close all;
clear;
clc;
close all;

rng(20);
%Run the simulation for a specific number of iterations
iterations = 1000;
videoFLag = 0
%% Set up the Robotarium object

N = 10;
K = 2;

% Working combinations N=10, K=3

start_in_corner = true
x_init = generate_initial_conditions(N); %,'Width',1.1,'Height',1.1,'Spacing', 0.35);
x_init = x_init - [min(x_init(1,:)) - (-1.6 + 0.2);min(x_init(2,:)) - (-1 + 0.2);0];
if start_in_corner
x_init(1, :) = rand(1, N)-1.6;
x_init(2, :) = rand(1, N) - 1;
else
x_init(1, :) = rand(1, N)*3.2 - 1.6;
x_init(2, :) = rand(1, N)*2 - 1;
end
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true,'InitialConditions',x_init);


%Initialize velocity vector
dxir = zeros(2, N);
dxid = zeros(2, K);

%Boundary
crs = [r.boundaries(1), r.boundaries(3);
    r.boundaries(1), r.boundaries(4);
    r.boundaries(2), r.boundaries(4);
    r.boundaries(2), r.boundaries(3)];

initial_drone_pose = rand(3, K);
initial_drone_pose(1, :) = initial_drone_pose(1, :)*(max(crs(:, 1))-min(crs(:, 1))) + min(crs(:,1));
initial_drone_pose(2, :) = initial_drone_pose(2, :)*(max(crs(:, 2))-min(crs(:, 2))) + min(crs(:,2));
initial_drone_pose(3, :) = initial_drone_pose(3, :)*2*pi - pi;
% crs = [r.boundaries(1), r.boundaries(3);
%        r.boundaries(1), r.boundaries(4);
%        1/3*r.boundaries(1), r.boundaries(4);
%        1/3*r.boundaries(1), 0;
%        1/3*r.boundaries(2), 0;
%        1/3*r.boundaries(2), r.boundaries(4);
%        r.boundaries(2), r.boundaries(4);
%        r.boundaries(2), r.boundaries(3)];

%Gausian Setup
center = [-1 1;0 0];
sigma = .15*eye(2);
detSigma = det(sigma);

sigmaR = .02*eye(2);
detSigmaR = det(sigmaR);
%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics();
% Single-integrator barrier certificates
uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary();
si_barrier_cert_boundary = create_si_barrier_certificate();
% Single-integrator position controller
motion_controller = create_si_position_controller('XVelocityGain', 5, 'YVelocityGain', 5, 'VelocityMagnitudeLimit', 1);
drone_motion_controller = create_si_position_controller('XVelocityGain', 10, 'YVelocityGain', 10, 'VelocityMagnitudeLimit', 10);

%% Plotting Setup

marker_size = determine_marker_size(r, 0.08);


x = r.get_poses();
xd = initial_drone_pose(1:2, :);

verCellHandle = zeros(N,1);
verCellHandleAerial = zeros(K,1);
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
    verCellHandleAerial(i)  = patch(xd(1,i),xd(2,i),cellColorsD(i,:),'FaceAlpha', 0.01, 'LineWidth', 4);
    % use color i  -- no robot assigned yet
    %verCellHandleD(i)  = patch(xd(1,i),xd(2,i),'EdgeColor','green','FaceColor','none','LineWidth',2);
    hold on
end

for i = 1:K
    aerialDroneHandle(i) = plot(xd(1,i), xd(2,i), 'r.', 'MarkerSize', 30)
    hold on;
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

if videoFLag 
    vid = VideoWriter('heterogeneous_coverage_control_video.mp4', 'MPEG-4');
    vid.Quality = 100;
    vid.FrameRate = 72;
    open(vid);
    writeVideo(vid, getframe(gcf));
end

xArrayD = linspace(r.boundaries(1),r.boundaries(2),plotRes);
yArrayD = linspace(r.boundaries(3),r.boundaries(4),plotRes);
zArrayD = ones(plotRes);
titl = sprintf('Heterogeneous Weight Lloyd with %d aerial and %d ground bots', K, N);
title(titl)
r.step();

inPosition = true;
drone_vel = 1;
%% Main Loop
in_pos = false;
weights = ones(1, K);

axis([-1.6 1.6 -1 1])
q = 1;

[v,c]=VoronoiBounded(xd(1, :)', xd(2, :)',crs);


%%
prev_cell = zeros(1, N)

res = 70

for t = 1:iterations
    if videoFLag && mod(t,10)                               % Record a video frame every 10 iterations
            writeVideo(vid, getframe(gcf)); 
    end

    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    poses = r.get_poses();
    x = poses(:, 1:N);
    
    % Convert to SI states
    xi = uni_to_si_states(x);
    
    
    
    %% Algorithm
     
    [Pxd, Pyd, cell_mass] = lloydsAlgorithmQ2(xd(1, :)', xd(2, :)', x(1, :)', x(2, :)', crs, verCellHandleAerial, res, center, sigma, detSigma);
    
    [Px, Py, prev_cell] = lloydsAlgorithmG(x(1,:)',x(2,:)',xd(1,:)',xd(2,:)', 0.1, cell_mass,  ...
        crs, verCellHandle, verCellHandleAerial, res, center, sigma, detSigma, true);
    
    
    dxir = motion_controller(x(1:2, :), [Px';Py']);
    
    dxid = drone_motion_controller(xd(1:2, :), [Pxd'; Pyd']);
    
    xd = dxid*.033 + xd;
    
    
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
    
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:(N), [dxu*in_pos]);%r.set_velocities(1:(N+K), [dxu*in_pos dxud]);
    
    
    
    %% Update Plot Handles
    
    for i = 1:N % color according to
        xD = [get(pathHandle(i),'XData'),x(1,i)];
        yD = [get(pathHandle(i),'YData'),x(2,i)];
        set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
    end
    
    
    set(aerialDroneHandle, 'XData', xd(1,:), 'YData', xd(2, :));
    
    set(centroidHandle,'XData',Px,'YData',Py);%plot centroid position
    
    
    r.step();
    if t == 1
        in_pos = true;
    end
    
    
    
end
if videoFLag; close(vid); end


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

function [Px, Py, cur_cell] = lloydsAlgorithmG(Px,Py,Pxd, Pyd, r,cell_mass, crs, verCellHandle, ...
    verCellHandleAerial, res, center, sigma, detSigma, plt)
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



% Apply lloyds

[vd,cd]=VoronoiBounded(Pxd, Pyd, crs);

[PX PXD] = meshgrid(Px, Pxd);
[PY PYD] = meshgrid(Py, Pyd);
distance_matrix = sqrt((PX-PXD).^2 + (PY - PYD).^2);
% The (i,j) position is the distance between quad i and robot j



%% calculate number of bots in each aerial partition
aerialCell = zeros(1, numel(Px));
for b = 1:numel(cd)
    aerialCell(inpolygon(Px, Py, vd(cd{b}, 1), vd(cd{b}, 2))) = b;
end

cur_cell = aerialCell;

%% calculate average density of each partition

for j = 1:numel(cd)
    
    if any(aerialCell == j)
        
        [v, c] = VoronoiBounded(Px(aerialCell == j), Py(aerialCell == j), [vd(cd{j}, 1), vd(cd{j}, 2)]);
        currX = Px(aerialCell == j);
        for i = 1:numel(c) %calculate the center of mass of each cell
            
            index = find(Px == currX(i));
            if ~isempty(c{i})
                %xVoronoi = linspace(min(v(c{i},1)) - r,max(v(c{i},1)) + r,res);  %linspace(min(v(c{i},1)),max(v(c{i},1)),res);
                %yVoronoi = linspace(min(v(c{i},2)) - r,max(v(c{i},2)) + r,res);  %linspace(min(v(c{i},2)),max(v(c{i},2)),res);
                xVoronoi = linspace(min(crs(:,1)), max(crs(:, 1)), res);
                yVoronoi = linspace(min(crs(:, 2)), max(crs(:, 2)), res);
                
                [coordsX, coordsY] = meshgrid(xVoronoi, yVoronoi);
                coords = [coordsX(:) coordsY(:)];
                
                in1 = inpolygon(coords(:,1),coords(:,2),v(c{i},1),v(c{i},2));
                in2 = sqrt((Px(index) - coords(:, 1)).^2 + (Py(index) - coords(:, 2)).^2) < r;
                in3 = inpolygon(coords(:, 1), coords(:, 2), vd(cd{j}, 1), vd(cd{j}, 2));
                
                xArrayIn = coords(in1 | (~in3 & in2), 1)';
                yArrayIn = coords(in1 | (~in3 & in2), 2)';
                
                positionMassSumX = 0;
                positionMassSumY = 0;
                totalMass = 0;
                
                for m = 1:length(xArrayIn)
                    zArrayIn = gaussC(xArrayIn(m),yArrayIn(m), sigma, detSigma, center);
                    positionMassSumX = positionMassSumX + zArrayIn*xArrayIn(m);
                    positionMassSumY = positionMassSumY + zArrayIn*yArrayIn(m);
                    totalMass = totalMass + zArrayIn;
                end
                
                positionMassSumX = positionMassSumX/(totalMass);
                positionMassSumY = positionMassSumY/(totalMass);
                cx = positionMassSumX;
                cy = positionMassSumY;
                cx = min(max(vd(cd{j}, 1)),max(min(vd(cd{j}, 1)), cx));
                cy = min(max(vd(cd{j}, 2)),max(min(vd(cd{j}, 2)), cy));
                
                if ~isnan(cx) 
                    Px(index) = cx;  %don't update if goal is outside the area
                    Py(index) = cy;
                   
                end
                

                if cell_mass(aerialCell(index)) < -1/numel(Px)
                    
                    important_cell_index = find(max(cell_mass) == cell_mass);
                    Px(index) = Pxd(important_cell_index(1));
                    Py(index) = Pyd(important_cell_index(1));
                    
                    
                end         
                    
                    %NEED TO FIX IF to go to high cell mass
                
                
                
                
                set(verCellHandle(index), 'XData',v(c{i},1),'YData',v(c{i},2));
                
            end
        end
        
        set(verCellHandleAerial(j), 'XData', vd(cd{j}, 1), 'YData', vd(cd{j},2) );
        
        
    end
end

end

function [Cx,Cy] = PolyCentroid(X,Y)
% POLYCENTROID returns the coordinates for the centroid of polygon with vertices X,Y
% The centroid of a non-self-intersecting closed polygon defined by n vertices (x0,y0), (x1,y1), ...,
% (xn?1,yn?1) is the point (Cx, Cy), where
% In these formulas, the vertices are assumed to be numbered in order of their occurrence along the polygon's
% perimeter, and the vertex ( xn, yn )
% is assumed to be the same as ( x0, y0 ). Note that if the points are numbered in clockwise order the area A,
% computed as above, will have a negative
% sign; but the centroid coordinates will be correct even in this case.http://en.wikipedia.org/wiki/Centroid
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


function [hit] =  hitBound(x,y, crs, ep)
max_x = max(crs(:, 1));
min_x = min(crs(:, 1));
max_y = max(crs(:, 2));
min_y = min(crs(:, 2));
hit = (x > max_x - ep) | (x < min_x + ep) | (y > max_y - ep) | (y < min_y + ep);
end

function [ordered_x, ordered_y] = poly2cw_custom(x,y)
cx = mean(x);
cy = mean(y);
a = atan2(y-cy, x -cx);

[~, order] = sort(a);
ordered_x = x(order);
ordered_y = y(order);
end

function [vec] = unit_vector(x, y)
vec = [x(:)'; y(:)'].*(1./sqrt(x.^2 + y.^2))

end

function val = gaussC(x, y, sigma, detSigma, center)
xc = center(1, :);
yc = center(2, :);
exponent = ((x-xc).^2/(sigma(1,1)) + (y-yc).^2/sigma(2,2))./(2);
amplitude = 1 / (sqrt(detSigma) * 2*pi);
val = sum(amplitude  .* exp(-exponent));
end


function [Px, Py, cell_mass] = lloydsAlgorithmQ2(Px,Py,Pxr, Pyr, crs, verCellHandle, res, center, sigma, detSigma)
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
rng(sd);
total_field_mass = 0;
total_robot_mass = 0;

xrange = max(crs(:,1)) - min(crs(:,1));
yrange = max(crs(:,2)) - min(crs(:,2));

% Apply LLYOD's Algorithm
[v,c]=VoronoiBounded(Px,Py,crs);
aerialCell = zeros(1, numel(Pxr));
for b = 1:numel(c)
    aerialCell(inpolygon(Pxr, Pyr, v(c{b}, 1), v(c{b}, 2))) = b;
end
cell_mass = zeros(1, numel(c));

for i = 1:numel(c) %calculate the center of mass of each cell

    xVoronoi = linspace(min(crs(:, 1)), max(crs(:,1)), res);
    yVoronoi = linspace(min(crs(:, 2)), max(crs(:,2)), res);
    coords = [];
    
    [X Y] = meshgrid(xVoronoi, yVoronoi);
    coords = [X(:) Y(:)];
    
    in = inpolygon(coords(:, 1),coords(:, 2),v(c{i},1),v(c{i},2));
    xArrayIn = coords(in, 1)';
    yArrayIn = coords(in, 2)';
    
    positionMassSumX = 0;
    positionMassSumY = 0;
    totalMass = 0;
    total_robot_mass = 0;
    total_field_mass = 0;
    
    for j = 1:length(xArrayIn)
        
        dx = (max(crs(:, 1)) - min(crs(:, 1)))/res;
        dy = (max(crs(:, 2)) - min(crs(:, 2)))/res;
        
        robotMass = gaussC(xArrayIn(j),yArrayIn(j), sigma, detSigma, [Pxr';Pyr'])/numel(Pxr);
        fieldMass = gaussC(xArrayIn(j),yArrayIn(j), sigma, detSigma, center)/(numel(center)/2);
        zArrayIn = 1;
        
        total_robot_mass = total_robot_mass + robotMass*dx*dy;
        total_field_mass = total_field_mass + fieldMass*dx*dy;
        positionMassSumX = positionMassSumX + zArrayIn*xArrayIn(j);
        positionMassSumY = positionMassSumY + zArrayIn*yArrayIn(j);
        totalMass = totalMass + zArrayIn;
    end
    
    total_robot_mass = numel(aerialCell(aerialCell == i))/numel(Pyr);
    cell_mass(i) = -total_robot_mass + total_field_mass;
 
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



for i = 1:numel(c) % update Voronoi cells
    set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
end
end