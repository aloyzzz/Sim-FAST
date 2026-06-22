%% Plot the deformation history and save as an MP4 video.

function Plot_Deformed_His_MP4(obj, Uhis, mp4FileName)

if nargin < 3
    mp4FileName = 'truss_deformation.mp4';
end

View1  = obj.viewAngle1;
View2  = obj.viewAngle2;
Vsize  = obj.displayRange;
Vratio = obj.displayRangeRatio;

assembly     = obj.assembly;
undeformNode = assembly.node.coordinates_mat;
barConnect   = assembly.bar.node_ij_mat;
barNum       = size(barConnect, 1);

actBarConnect = assembly.actBar.node_ij_mat;
actBarNum     = size(actBarConnect, 1);

nSteps = size(Uhis, 1);

% --- set up figure ---
h = figure;
set(h, 'color', 'white');
set(h, 'position', [obj.x0, obj.y0, obj.width, obj.height]);

% --- set up video writer ---
v = VideoWriter(mp4FileName, 'MPEG-4');
v.FrameRate = max(1, round(1 / obj.holdTime));
open(v);

for i = 1:nSteps
    clf
    view(View1, View2);
    set(gca, 'DataAspectRatio', [1 1 1]);
    set(gcf, 'color', 'white');

    if isscalar(Vsize)
        axis([-Vratio*Vsize Vsize -Vratio*Vsize Vsize -Vratio*Vsize Vsize]);
    else
        axis([Vsize(1) Vsize(2) Vsize(3) Vsize(4) Vsize(5) Vsize(6)]);
    end
    hold on;

    % Draw undeformed shape in light gray
    for j = 1:barNum
        n1 = undeformNode(barConnect(j,1), :);
        n2 = undeformNode(barConnect(j,2), :);
        line([n1(1),n2(1)], [n1(2),n2(2)], [n1(3),n2(3)], ...
             'Color', [0.75 0.75 0.75], 'LineStyle', '--');
    end
    for j = 1:actBarNum
        n1 = undeformNode(actBarConnect(j,1), :);
        n2 = undeformNode(actBarConnect(j,2), :);
        line([n1(1),n2(1)], [n1(2),n2(2)], [n1(3),n2(3)], ...
             'Color', [0.65 0.75 1.0], 'LineStyle', '--');
    end

    % Draw deformed shape
    tempU     = squeeze(Uhis(i, :, :));
    deformNode = undeformNode + tempU;

    for j = 1:barNum
        n1 = deformNode(barConnect(j,1), :);
        n2 = deformNode(barConnect(j,2), :);
        line([n1(1),n2(1)], [n1(2),n2(2)], [n1(3),n2(3)], ...
             'Color', 'k', 'LineWidth', 1.5);
    end
    for j = 1:actBarNum
        n1 = deformNode(actBarConnect(j,1), :);
        n2 = deformNode(actBarConnect(j,2), :);
        line([n1(1),n2(1)], [n1(2),n2(2)], [n1(3),n2(3)], ...
             'Color', 'b', 'LineWidth', 3);
    end

    % Draw deformed nodes
    scatter3(deformNode(:,1), deformNode(:,2), deformNode(:,3), ...
             30, 'k', 'filled');

    title(sprintf('Step %d / %d', i, nSteps), 'FontSize', 12);
    axis off;
    drawnow;

    writeVideo(v, getframe(h));
end

close(v);
close(h);

fprintf('Saved: %s\n', mp4FileName);
end
