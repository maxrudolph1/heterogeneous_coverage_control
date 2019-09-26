
clf;
close all;
crs = [-1.60000000000000,-1;-1.60000000000000,1;1.60000000000000,1;1.60000000000000,-1]
for k = 0:1/100:3*pi
    n = 3;
    points_x = [-.75 .75 0];
    cellColors = cool(n);
    points_y = [-.4 -.4 .4];
    axis([-1.6 1.6 -1 1]);
    weights = [cos(k) + 4, cos(2*k) + 4, cos(3*k)+4]';
    [V C] = power_bounded(points_x', points_y', weights, crs);
    
    r = weights/10;
    t = 0:.01:2*pi;
    poly = zeros(numel(t), 2*numel(points_x));
    poly(:,1:2:end) = sin(t)'*r'+(points_x'*ones(1,numel(t)))';
    poly(:,2:2:end) = cos(t)'*r'+(points_y'*ones(1,numel(t)))';
    
    
    plot(points_x, points_y, '*')
    hold on;
    for i = 1:n
        patch(V(C{i},1),V(C{i},2),cellColors(i,:),'FaceAlpha', 0.3);
    end
    
    for i = 0:(numel(r)-1)
        patch(poly(:, i*2+1), poly(:, 2*i+2), cellColors(i+1, :), 'FaceAlpha', 0.3);
    end
    axis([-1.6 1.6 -1 1]);
    pause(.01)
    clf;
end