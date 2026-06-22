%% Hex_Center_Ring.m
%
% Dynamic response: 1 center hexagon + 6 ring hexagons.
% 12 joints total (same type for all):
%   6 center-ring : center hex vertex k  <-->  ring hex k inner vertex
%   6 ring-ring   : ring hex k vertex j6 <-->  ring hex k+1 vertex j2
%
% d_ring = 3R  =>  each hex-hex gap = R  (same prong length as 3-hex case).
%
% Y-joint   : hub node at gap midpoint + 2 prong bars per connection.
% Triangle  : single flexible bar across each gap (no hub node).
%
% Ring hex 4 (angle 180°, opposite loading direction) is clamped.
% Ring hex 1 (angle 0°, right side) receives a +X force impulse.
% Actuation: joint bars are linear actuators with prescribed rest lengths.

clear; close all; clc;

%% --- Joint type ------------------------------------------------------------
%   'Y'        : hub node + 2 prong bars per joint
%   'triangle' : single flexible bar per joint

joint_type = 'triangle';

%% --- Paths -----------------------------------------------------------------
baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Solver')));

%% --- Parameters ------------------------------------------------------------
R           = 0.50;      % hexagon circumradius (m)
barA        = 1e-4;      % bar cross-section area (m^2)
E_panel     = 70e9;      % panel Young's modulus (Pa) — aluminium
E_joint     = 1e8;       % joint Young's modulus (Pa) — semi-rigid

m_node      = 10.0;      % lumped mass per node (kg)

F_mag       = 0;         % external force (N); 0 = pure actuation demo
t_force     = 2.0;       % force-on duration (s)
t_total     = 12.0;      % total simulation time (s)
dt          = 0.01;      % time step (s)

alpha_d     = 0.10;      % Rayleigh mass-proportional damping
beta_d      = 0.0;

skipFr      = 5;
fps         = 20;

%% --- Geometry --------------------------------------------------------------
% Center hex (h=1): flat-top, vertex j at angle (j-1)*60°.
% Ring hex k (h=k+1, k=1..6): center at d_ring in direction theta_k=(k-1)*60°;
%   inner vertex (j=1) points back toward origin (angle theta_k+180°).
% With d_ring=3R: center hex vertex k is at R in direction theta_k;
%   ring hex k inner vertex is at 2R in same direction; gap = R.

nRing  = 6;
nHex   = 1 + nRing;
d_ring = 3 * R;

use_hub = strcmp(joint_type, 'Y');

n_panel_raw = nHex * 6;
max_raw     = n_panel_raw + 12 + 5;   % panel nodes + possible hub nodes
raw_coords  = zeros(max_raw, 3);
raw_hex     = zeros(nHex, 6);

% Center hex (h=1)
for j = 1:6
    raw_coords(j, :) = [R*cos((j-1)*pi/3), 0, R*sin((j-1)*pi/3)];
    raw_hex(1, j) = j;
end

% Ring hexes (h=k+1 for k=1..nRing)
for k = 1:nRing
    theta_k = (k-1) * pi/3;
    cx = d_ring * cos(theta_k);
    cz = d_ring * sin(theta_k);
    for j = 1:6
        phi_j = (theta_k + pi) + (j-1)*pi/3;
        idx   = 6 + (k-1)*6 + j;
        raw_coords(idx, :) = [cx + R*cos(phi_j), 0, cz + R*sin(phi_j)];
        raw_hex(k+1, j) = idx;
    end
end
n_placed = n_panel_raw;

% Hub nodes for Y-joint (midpoints of each joint gap)
cr_hub_raw = zeros(nRing, 1);  % center-ring hub raw indices
rr_hub_raw = zeros(nRing, 1);  % ring-ring hub raw indices

if use_hub
    for k = 1:nRing
        theta_k = (k-1)*pi/3;
        n_placed = n_placed + 1;
        cr_hub_raw(k) = n_placed;
        raw_coords(n_placed, :) = [1.5*R*cos(theta_k), 0, 1.5*R*sin(theta_k)];
    end
    for k = 1:nRing
        k2 = mod(k, nRing) + 1;
        p1 = raw_coords(raw_hex(k+1, 6), :);    % ring hex k vertex j=6
        p2 = raw_coords(raw_hex(k2+1, 2), :);   % ring hex k+1 vertex j=2
        n_placed = n_placed + 1;
        rr_hub_raw(k) = n_placed;
        raw_coords(n_placed, :) = (p1 + p2) / 2;
    end
end
n_raw = n_placed;

% Merge coincident vertices
tol = R * 1e-6;
[node_coords, ~, ic] = uniquetol(raw_coords(1:n_raw, :), tol, ...
                                 'ByRows', true, 'DataScale', 1);

hex_nodes = zeros(nHex, 6);
for h = 1:nHex
    for j = 1:6
        hex_nodes(h, j) = ic(raw_hex(h, j));
    end
end
center_nodes     = hex_nodes(1, :);                    % center hex vertices
inner_ring_nodes = hex_nodes(2:end, 1);                % ring hex inner vertices (6×1)

if use_hub
    cr_hub = ic(cr_hub_raw);
    rr_hub = ic(rr_hub_raw);
end

nNodes = size(node_coords, 1);
fprintf('Joint type   : %s\n', joint_type);
fprintf('Unique nodes : %d\n', nNodes);

%% --- Bar connectivity ------------------------------------------------------
% Panel bars: 6 perimeter + 3 fan-diagonals per hex (no sharing; hexes have gaps).
% Joint bars: per joint type, see below.

max_bars = nHex*9 + 50;
raw_bars = zeros(max_bars, 2);
is_joint_flag = false(max_bars, 1);
nb = 0;

for h = 1:nHex
    v = hex_nodes(h, :);
    for j = 1:6
        nb = nb+1; raw_bars(nb,:) = [v(j), v(mod(j,6)+1)];
    end
    for j = 2:4
        nb = nb+1; raw_bars(nb,:) = [v(1), v(j+1)];
    end
end

% Joint bars
if use_hub
    for k = 1:nRing           % center-ring: hub in the middle
        nb = nb+1; raw_bars(nb,:) = [center_nodes(k),      cr_hub(k)];  is_joint_flag(nb)=true;
        nb = nb+1; raw_bars(nb,:) = [cr_hub(k), inner_ring_nodes(k)];   is_joint_flag(nb)=true;
    end
    for k = 1:nRing           % ring-ring: hub in the middle
        k2 = mod(k, nRing) + 1;
        nb = nb+1; raw_bars(nb,:) = [hex_nodes(k+1,6),  rr_hub(k)]; is_joint_flag(nb)=true;
        nb = nb+1; raw_bars(nb,:) = [rr_hub(k), hex_nodes(k2+1,2)]; is_joint_flag(nb)=true;
    end
else
    for k = 1:nRing           % center-ring: direct bar
        nb = nb+1; raw_bars(nb,:) = [center_nodes(k), inner_ring_nodes(k)]; is_joint_flag(nb)=true;
    end
    for k = 1:nRing           % ring-ring: direct bar
        k2 = mod(k, nRing) + 1;
        nb = nb+1; raw_bars(nb,:) = [hex_nodes(k+1,6), hex_nodes(k2+1,2)]; is_joint_flag(nb)=true;
    end
end

raw_bars_s      = sort(raw_bars(1:nb,:), 2);
flag_s          = is_joint_flag(1:nb);
[bar_conn, ~, ic_bar] = unique(raw_bars_s, 'rows');
nBars = size(bar_conn, 1);

is_joint = false(nBars, 1);
for b = 1:nb
    if flag_s(b), is_joint(ic_bar(b)) = true; end
end
fprintf('Unique bars  : %d  (joint: %d)\n', nBars, sum(is_joint));

%% --- Identify actuation bar indices ----------------------------------------
% act_ids_cr(k): bar index (or pair of bar indices) for joint k.
% For Y-joint: 2 prong bars per joint; for triangle: 1 bar per joint.
% Store all joint bar indices in act_ids (nJoint bars × 1).

act_ids = find(is_joint);   % all joint bar indices (ordered by bar_conn)
nActBars = numel(act_ids);
fprintf('Actuated bars: %d\n', nActBars);

%% --- Elements & assembly ---------------------------------------------------
node = Elements_Nodes;
node.coordinates_mat = node_coords;
node.mass_vec        = m_node * ones(nNodes, 1);

bar = Vec_Elements_Bars;
bar.node_ij_mat = bar_conn;
bar.A_vec       = barA * ones(nBars, 1);
E_vec           = E_panel * ones(nBars, 1);
E_vec(is_joint) = E_joint;
bar.E_vec       = E_vec;

assembly = Assembly_Hex_Origami;
assembly.node = node;
assembly.bar  = bar;
assembly.Initialize_Assembly();

%% --- Support conditions ----------------------------------------------------
% Ring hex 4 (angle 180°, h=5) fully clamped in XZ plane.
% All nodes: Y fixed (flat structure).

ring4_nodes = unique(hex_nodes(5, :));   % ring hex 4 = h=5

supp      = zeros(nNodes, 4);
supp(:,1) = (1:nNodes)';
supp(:,3) = 1;                   % Y fixed for all

for n = ring4_nodes(:)'
    supp(n,2) = 1;               % X fixed
    supp(n,4) = 1;               % Z fixed
end

% Also clamp hub nodes adjacent to ring hex 4 (if Y-joint)
if use_hub
    % cr_hub(4) connects center to ring hex 4
    supp(cr_hub(4),2) = 1; supp(cr_hub(4),4) = 1;
    % rr_hub(3) and rr_hub(4) connect ring hex 4 to ring hex 3 and 5
    supp(rr_hub(3),2) = 1; supp(rr_hub(3),4) = 1;
    supp(rr_hub(4),2) = 1; supp(rr_hub(4),4) = 1;
end

%% --- External force --------------------------------------------------------
% +X force on outer nodes of ring hex 1 (h=2).

outer_ring1 = setdiff(hex_nodes(2,:), inner_ring_nodes(1));
nFN         = numel(outer_ring1);

totalSteps = round(t_total / dt);
stepForce  = round(t_force / dt);

Fext = zeros(totalSteps, nNodes, 3);
for n = outer_ring1(:)'
    Fext(1:stepForce, n, 1) = F_mag / nFN;
end

%% --- Actuation trajectory --------------------------------------------------
% Natural rest lengths (set by Initialize_Assembly).
L0_nat = bar.L0_vec(act_ids);       % (nActBars × 1)

strain_act = -0.15;                  % fractional contraction per actuator
t_ramp     = 1.5;                    % ramp time (s)
t_hold     = 1.0;                    % hold time (s)

ramp_steps = round(t_ramp / dt);
hold_steps = round(t_hold / dt);
cycle      = 2*ramp_steps + hold_steps;

% L0_his: (totalSteps × nActBars) — default = natural length
L0_his = repmat(L0_nat', totalSteps, 1);

for b = 1:nActBars
    dL    = strain_act * L0_nat(b);
    shift = (b-1) * cycle;
    for s = 1:ramp_steps
        idx = shift + s;
        if idx <= totalSteps
            L0_his(idx, b) = L0_nat(b) + dL * (s/ramp_steps);
        end
    end
    for s = 1:hold_steps
        idx = shift + ramp_steps + s;
        if idx <= totalSteps
            L0_his(idx, b) = L0_nat(b) + dL;
        end
    end
    for s = 1:ramp_steps
        idx = shift + ramp_steps + hold_steps + s;
        if idx <= totalSteps
            L0_his(idx, b) = L0_nat(b) + dL*(1 - s/ramp_steps);
        end
    end
end

act_struct.bar_ids = act_ids;
act_struct.L0_nat  = L0_nat;
act_struct.L0_his  = L0_his;

%% --- Dynamic simulation ----------------------------------------------------
caa                   = Solver_CAA_Dynamics;
caa.assembly          = assembly;
caa.supp              = supp;
caa.dt                = dt;
caa.Fext              = Fext;
caa.alpha             = alpha_d;
caa.beta              = beta_d;
caa.rotSprTargetAngle = [];
caa.actuation         = act_struct;

fprintf('Running %d steps (%.1f s)...\n', totalSteps, t_total);
Uhis = caa.Solve();
fprintf('Done.\n');

%% --- Generate MP4 ----------------------------------------------------------
Uhis_sub = Uhis(1:skipFr:end, :, :);
nFrames  = size(Uhis_sub, 1);

mp4File = fullfile(fileparts(mfilename('fullpath')), ...
                   sprintf('hex_center_ring_%s.mp4', lower(joint_type)));
vid = VideoWriter(mp4File, 'MPEG-4');
vid.FrameRate = fps;
open(vid);

fig = figure('color','white','position',[0 0 900 850]);

% Colors: center=teal, ring hex 1=gold, ring hex 4=dark (clamped), others=steel
hex_face        = repmat([0.30 0.55 0.80], nHex, 1);  % default steel blue
hex_face(1,:)   = [0.10 0.70 0.65];   % center — teal
hex_face(2,:)   = [1.00 0.82 0.10];   % ring hex 1 — gold (loaded)
hex_face(5,:)   = [0.25 0.25 0.25];   % ring hex 4 — dark (clamped)

act_cmap       = [1 0.15 0.15; 0.10 0.75 0.20; 0.15 0.35 1.0];
act_ratio_pts  = [0.80, 1.00, 1.20];

pad  = R * 0.5;
xAll = node_coords(:,1);  zAll = node_coords(:,3);
xLim = [min(xAll)-pad, max(xAll)+pad];
zLim = [min(zAll)-pad, max(zAll)+pad];

for fi = 1:nFrames
    clf; hold on; axis equal off;
    xlim(xLim); ylim(zLim);
    set(gcf,'color','white');

    step       = min((fi-1)*skipFr + 1, totalSteps);
    tempU      = squeeze(Uhis_sub(fi,:,:));
    deformNode = node_coords + tempU;

    % Actuation ratios for all joint bars
    ratio = L0_his(step, :) ./ L0_nat';
    ratio_cl = max(act_ratio_pts(1), min(act_ratio_pts(end), ratio));
    bar_clr  = interp1(act_ratio_pts, act_cmap, ratio_cl);  % (nActBars × 3)

    % --- Undeformed ghost ---------------------------------------------------
    for h = 1:nHex
        vxz = node_coords(hex_nodes(h,:), [1 3]);
        patch(vxz(:,1), vxz(:,2), [0.93 0.93 0.93], ...
              'EdgeColor',[0.80 0.80 0.80], 'FaceAlpha',0.35, ...
              'LineWidth',0.5, 'LineStyle','--');
    end
    % Ghost joint bars
    for b = 1:nActBars
        n1 = bar_conn(act_ids(b),1);  n2 = bar_conn(act_ids(b),2);
        p1 = node_coords(n1,[1 3]);   p2 = node_coords(n2,[1 3]);
        plot([p1(1) p2(1)],[p1(2) p2(2)],'--','Color',[0.80 0.80 0.80],'LineWidth',0.8);
    end

    % --- Deformed panels ----------------------------------------------------
    for h = 1:nHex
        vxz = deformNode(hex_nodes(h,:), [1 3]);
        patch(vxz(:,1), vxz(:,2), hex_face(h,:), ...
              'EdgeColor','k', 'FaceAlpha',1.0, 'LineWidth',1.5);
    end

    % --- Deformed joint bars (colored by actuation ratio) -------------------
    for b = 1:nActBars
        n1 = bar_conn(act_ids(b),1);  n2 = bar_conn(act_ids(b),2);
        p1 = deformNode(n1,[1 3]);    p2 = deformNode(n2,[1 3]);
        plot([p1(1) p2(1)],[p1(2) p2(2)],'-','Color',bar_clr(b,:),'LineWidth',3.5);
    end

    % Hub node markers (Y-joint only)
    if use_hub
        all_hubs = [cr_hub; rr_hub];
        for b = 1:numel(all_hubs)
            hxz = deformNode(all_hubs(b), [1 3]);
            plot(hxz(1), hxz(2), 'o', 'MarkerSize',6, ...
                 'MarkerFaceColor',[0.3 0.3 0.3],'MarkerEdgeColor','k','LineWidth',1.0);
        end
    end

    % Force arrow
    realTime = fi * skipFr * dt;
    if realTime <= t_force && F_mag > 0
        fc = mean(deformNode(outer_ring1,[1 3]),1);
        quiver(fc(1), fc(2), R*0.5, 0, 0, 'r','LineWidth',2.5,'MaxHeadSize',0.8);
    end

    title(sprintf('%s joint  |  1+6 hex ring  |  t = %.2f s', joint_type, realTime), ...
          'FontSize',11);
    drawnow;
    writeVideo(vid, getframe(fig));
end

close(vid); close(fig);
fprintf('Saved: %s\n', mp4File);

%% --- Displacement time-history ---------------------------------------------
time = (0:totalSteps-1)*dt;

% Centroid X-displacement for each ring hex + center hex
u_center = mean(squeeze(Uhis(:, center_nodes, 1)), 2);
u_ring   = zeros(totalSteps, nRing);
for k = 1:nRing
    u_ring(:,k) = mean(squeeze(Uhis(:, hex_nodes(k+1,:), 1)), 2);
end

ring_clrs = [1.00 0.82 0.10;   % ring 1 — gold
             0.30 0.55 0.80;
             0.30 0.55 0.80;
             0.25 0.25 0.25;   % ring 4 — dark (clamped)
             0.30 0.55 0.80;
             0.30 0.55 0.80];

figure('Position',[100 100 960 520]);

subplot(2,1,1); hold on; grid on;
plot(time, u_center*1e3, 'Color',[0.10 0.70 0.65], 'LineWidth',1.5, 'DisplayName','Center');
for k = 1:nRing
    plot(time, u_ring(:,k)*1e3, 'Color',ring_clrs(k,:), 'LineWidth',1.2, ...
         'DisplayName',sprintf('Ring %d',k));
end
ylabel('Centroid X-disp (mm)');
title(sprintf('Panel response — %s joint, 1+6 hex', joint_type));
legend('Location','best','FontSize',8);

subplot(2,1,2); hold on; grid on;
act_c = lines(nActBars);
for b = 1:nActBars
    plot(time, L0_his(:,b)./L0_nat(b), 'Color',act_c(b,:), 'LineWidth',1.0);
end
yline(1,'k--','Natural','FontSize',8);
ylabel('L_0(t)/L_{0,nat}');
xlabel('Time (s)');
title(sprintf('Actuator rest-length ratios (%d joint bars)', nActBars));
