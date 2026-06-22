%% Hex_Ring_Dynamic.m
%
% Dynamic response of an A-ring: 6 solid hexagonal solar panels in a ring.
%
% Each panel is modeled as a triangulated bar mesh.
% Bars WITHIN a panel use a high (aluminum-like) Young's modulus so each
% hexagon behaves as a near-rigid body.
% Bars AT THE INTERFACE between adjacent panels use a much lower modulus,
% representing flexible inter-panel joints.
%
% A lateral in-plane force is applied to panel 1 for t_force seconds,
% then removed; free vibration is observed for the remainder.
% Results exported as hex_ring_response.mp4.

clear; close all; clc;

%% --- Paths -----------------------------------------------------------------
baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Solver')));

%% --- Parameters ------------------------------------------------------------
R           = 0.50;      % hexagon circumradius (m) — satellite panel scale
barA        = 1e-4;      % bar cross-section area (m^2)
E_panel     = 70e9;      % Young's modulus inside each panel (Pa) — aluminium
E_joint     = 5e5;       % Young's modulus at inter-panel joints (Pa) — flexible

m_node      = 10.0;      % lumped mass per node (kg)

F_mag       = 40.0;       % applied force (N)
t_force     = 2.0;       % force duration (s)
t_total     = 10.0;      % total simulation time (s)
dt          = 0.01;      % time step (s)

alpha_d     = 0.10;      % Rayleigh mass-proportional damping
beta_d      = 0.0;       % Rayleigh stiffness-proportional damping

skipFr      = 5;         % frame subsampling for MP4
fps         = 20;        % MP4 playback frame rate

%% --- Hexagonal-ring geometry -----------------------------------------------
% 6 pointy-top hexagons; ring radius d_ring = R*sqrt(3).
% All nodes lie in the XZ plane (Y = 0).

d_ring = R * sqrt(3);

raw_coords = zeros(36, 3);
raw_hex    = zeros(6, 6);   % raw_hex(k,j) = raw vertex index for hex k, vertex j

for k = 1:6
    theta_k = (k-1) * pi/3;
    cx = d_ring * cos(theta_k);
    cz = d_ring * sin(theta_k);
    for j = 1:6
        phi_j = pi/2 + (j-1)*pi/3;
        idx = (k-1)*6 + j;
        raw_coords(idx,:) = [cx + R*cos(phi_j),  0,  cz + R*sin(phi_j)];
        raw_hex(k,j) = idx;
    end
end

% Merge shared vertices between adjacent panels
tol = R * 1e-6;
[node_coords, ~, ic] = uniquetol(raw_coords, tol, 'ByRows', true, 'DataScale', 1);
hex_nodes = reshape(ic(raw_hex(:)), 6, 6);   % hex_nodes(k,j) = unique node index

nNodes = size(node_coords, 1);
fprintf('Unique nodes: %d  (expected 24)\n', nNodes);

%% --- Bar connectivity ------------------------------------------------------
% Fan-triangulate each hexagon (4 triangles, all edges kept).
% Deduplicate shared edges.

raw_bars = zeros(6*9, 2);
nb = 0;
for k = 1:6
    v = hex_nodes(k,:);
    for j = 1:6                       % perimeter edges
        nb = nb+1;
        raw_bars(nb,:) = [v(j), v(mod(j,6)+1)];
    end
    for j = 2:4                       % internal diagonal edges
        nb = nb+1;
        raw_bars(nb,:) = [v(1), v(j+1)];
    end
end
raw_bars = sort(raw_bars(1:nb,:), 2);
bar_conn = unique(raw_bars, 'rows');
nBars    = size(bar_conn, 1);
fprintf('Unique bars: %d\n', nBars);

%% --- Identify inter-panel joint bars ---------------------------------------
% For each adjacent pair (hex k, hex k+1) find the 2 shared nodes;
% the bar connecting them is the joint bar.

is_joint = false(nBars, 1);
for k = 1:6
    k2 = mod(k, 6) + 1;
    shared = intersect(hex_nodes(k,:), hex_nodes(k2,:));
    if numel(shared) == 2
        bp = sort(shared(:)');
        row = find(bar_conn(:,1)==bp(1) & bar_conn(:,2)==bp(2), 1);
        if ~isempty(row)
            is_joint(row) = true;
        end
    end
end
fprintf('Joint bars: %d  (expected 6)\n', sum(is_joint));

%% --- Elements & assembly ---------------------------------------------------
node = Elements_Nodes;
node.coordinates_mat = node_coords;
node.mass_vec = m_node * ones(nNodes, 1);

bar = Vec_Elements_Bars;
bar.node_ij_mat = bar_conn;
bar.A_vec = barA * ones(nBars, 1);

E_vec = E_panel * ones(nBars, 1);   % stiff inside each panel
E_vec(is_joint) = E_joint;          % soft at inter-panel joints
bar.E_vec = E_vec;

assembly = Assembly_Hex_Origami;
assembly.node = node;
assembly.bar  = bar;
assembly.Initialize_Assembly();

%% --- Support conditions ----------------------------------------------------
% Fix panel 4 (opposite the loaded panel) in-plane.
% Constrain Y for every node (flat ring stays flat).

hex4_nodes = unique(hex_nodes(4,:));

supp = zeros(nNodes, 4);
supp(:,1) = (1:nNodes)';
supp(:,3) = 1;                  % all nodes: Y fixed (out-of-plane)

for n = hex4_nodes(:)'
    supp(n,2) = 1;              % X fixed
    supp(n,4) = 1;              % Z fixed
end

%% --- External force --------------------------------------------------------
% Apply +X force on the outer-unique nodes of panel 1.

shared_nbrs = unique([hex_nodes(2,:), hex_nodes(6,:)]);
outer_hex1  = setdiff(hex_nodes(1,:), shared_nbrs);
nFN         = numel(outer_hex1);

totalSteps = round(t_total / dt);
stepForce  = round(t_force / dt);

Fext = zeros(totalSteps, nNodes, 3);
for n = outer_hex1(:)'
    Fext(1:stepForce, n, 1) = F_mag / nFN;
end

%% --- Dynamic simulation ---------------------------------------------------
caa = Solver_CAA_Dynamics;
caa.assembly = assembly;
caa.supp     = supp;
caa.dt       = dt;
caa.Fext     = Fext;
caa.alpha    = alpha_d;
caa.beta     = beta_d;
caa.rotSprTargetAngle = [];

fprintf('Running %d steps (%.1f s)...\n', totalSteps, t_total);
Uhis = caa.Solve();
fprintf('Done.\n');

%% --- Generate MP4 ---------------------------------------------------------
Uhis_sub = Uhis(1:skipFr:end, :, :);
nFrames  = size(Uhis_sub, 1);

mp4File = fullfile(fileparts(mfilename('fullpath')), 'hex_ring_response.mp4');
vid = VideoWriter(mp4File, 'MPEG-4');
vid.FrameRate = fps;
open(vid);

fig = figure('color','white','position',[0 0 800 750]);

% Panel colors: gold = loaded (hex 1), dark = clamped (hex 4), steel = rest
hex_face = [1.00 0.82 0.10;   % 1 — gold
            0.30 0.55 0.80;   % 2 — steel blue
            0.30 0.55 0.80;   % 3
            0.25 0.25 0.25;   % 4 — dark (clamped)
            0.30 0.55 0.80;   % 5
            0.30 0.55 0.80];  % 6

pad  = R * 0.2;
xAll = node_coords(:,1);  zAll = node_coords(:,3);
xLim = [min(xAll)-R-pad, max(xAll)+R+pad];
zLim = [min(zAll)-R-pad, max(zAll)+R+pad];

for fi = 1:nFrames
    clf; hold on; axis equal off;
    xlim(xLim); ylim(zLim);
    set(gcf,'color','white');

    tempU      = squeeze(Uhis_sub(fi,:,:));
    deformNode = node_coords + tempU;

    % Undeformed ghost
    for k = 1:6
        vxz = node_coords(hex_nodes(k,:), [1 3]);
        patch(vxz(:,1), vxz(:,2), [0.93 0.93 0.93], ...
              'EdgeColor',[0.75 0.75 0.75], 'FaceAlpha',0.4, ...
              'LineWidth',0.5, 'LineStyle','--');
    end

    % Deformed panels — filled, solid-looking
    for k = 1:6
        vxz = deformNode(hex_nodes(k,:), [1 3]);
        patch(vxz(:,1), vxz(:,2), hex_face(k,:), ...
              'EdgeColor','k', 'FaceAlpha',1.0, 'LineWidth',1.8);
    end

    % Highlight joint bars
    for k = 1:6
        k2 = mod(k,6)+1;
        sh = intersect(hex_nodes(k,:), hex_nodes(k2,:));
        if numel(sh)==2
            p1 = deformNode(sh(1),[1 3]);
            p2 = deformNode(sh(2),[1 3]);
            plot([p1(1) p2(1)],[p1(2) p2(2)],'w-','LineWidth',2.5);
        end
    end

    % Force arrow while force is active
    realTime = fi * skipFr * dt;
    if realTime <= t_force
        fc = mean(deformNode(outer_hex1, [1 3]), 1);
        quiver(fc(1)+R*0.55, fc(2), -R*0.45, 0, 0, 'r', ...
               'LineWidth',2.5, 'MaxHeadSize',1.0);
        phase = sprintf('F = %.1f N applied', F_mag);
    else
        phase = 'Free vibration';
    end

    title(sprintf('t = %.2f s  |  %s', realTime, phase), 'FontSize',12);
    drawnow;
    writeVideo(vid, getframe(fig));
end

close(vid); close(fig);
fprintf('Saved: %s\n', mp4File);

%% --- Displacement time-history ---------------------------------------------
time   = (0:totalSteps-1)*dt;
u_hex1 = mean(squeeze(Uhis(:, hex_nodes(1,:), 1)), 2);   % X-centroid of panel 1

figure;
plot(time, u_hex1*1e3, 'b', 'LineWidth',1.5); hold on;
xline(t_force,'r--','Force off','LabelVerticalAlignment','bottom','FontSize',10);
xlabel('Time (s)');
ylabel('Panel 1 centroid X-displacement (mm)');
title('Dynamic response — panel 1');
grid on;
