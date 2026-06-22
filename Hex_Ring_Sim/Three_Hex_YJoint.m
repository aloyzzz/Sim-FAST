%% Three_Hex_YJoint.m
%
% Dynamic response of 3 solid hexagonal panels connected at the center by
% either a Y-joint or a triangle joint.
%
% Geometry: 3 pointy-top hexagons at 0°, 120°, 240° around the origin.
%   Each hexagon is oriented so one vertex (the "inner vertex") aims inward.
%
% Y-JOINT      — a single hub node at the origin, connected to each inner
%                vertex by a flexible prong bar (3 bars total).
%
% TRIANGLE     — the 3 inner vertices are connected directly to each other
%                by 3 flexible bars, forming a central equilateral triangle.
%                No extra node is needed.
%
% Panel bars   : stiff (aluminium-like E)
% Joint bars   : soft (E_joint) — the compliant connection
%
% Panel 1 (right, gold)  receives a lateral outward (+X) impulse.
% Panel 3 (lower-left)   is clamped as the ground fixture.

clear; close all; clc;

%% --- Joint type ------------------------------------------------------------
%   'Y'        : 3-pronged hub (single central node + 3 prong bars)
%   'triangle' : triangular frame (3 bars between the inner vertices)

joint_type = 'Y';

%% --- Paths -----------------------------------------------------------------
baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Solver')));

%% --- Parameters ------------------------------------------------------------
R           = 0.50;      % hexagon circumradius (m)
barA        = 1e-4;      % bar cross-section area (m^2)
E_panel     = 70e9;      % panel Young's modulus (Pa) — aluminium
E_joint     = 1e8;       % joint Young's modulus (Pa) — semi-rigid (100 MPa)

m_node      = 10.0;      % lumped mass per node (kg)

F_mag       = 0;      % applied force magnitude (N)
t_force     = 2.0;       % force-on duration (s)
t_total     = 10.0;      % total simulation time (s)
dt          = 0.01;      % time step (s)

alpha_d     = 0.10;      % Rayleigh mass-proportional damping
beta_d      = 0.0;       % Rayleigh stiffness-proportional damping

skipFr      = 5;         % frame subsampling for MP4
fps         = 20;        % MP4 playback frame rate

%% --- Hexagon geometry (shared by both joint types) -------------------------
% Hex centers at distance d_ring = 2R.
% Inner vertex of hex k sits at distance R from origin along theta_k.
% Triangle joint side length = R*sqrt(3)  (equilateral).
% Y-joint prong length       = R.

nHex   = 3;
d_ring = 2 * R;

n_hex_raw  = nHex * 6;
raw_coords = zeros(n_hex_raw + 1, 3);   % +1 slot for optional hub
raw_hex    = zeros(nHex, 6);
inner_raw  = zeros(nHex, 1);

for k = 1:nHex
    theta_k = (k-1) * 2*pi/3;
    cx = d_ring * cos(theta_k);
    cz = d_ring * sin(theta_k);
    for j = 1:6
        phi_j = (theta_k + pi) + (j-1)*pi/3;   % vertex 1 aims toward origin
        idx   = (k-1)*6 + j;
        raw_coords(idx,:) = [cx + R*cos(phi_j),  0,  cz + R*sin(phi_j)];
        raw_hex(k,j) = idx;
    end
    inner_raw(k) = (k-1)*6 + 1;
end

use_hub = strcmp(joint_type, 'Y');
hub_raw_idx = n_hex_raw + 1;
if use_hub
    raw_coords(hub_raw_idx,:) = [0, 0, 0];
    n_raw = hub_raw_idx;
else
    n_raw = n_hex_raw;
end

% Merge coincident vertices
tol = R * 1e-6;
[node_coords, ~, ic] = uniquetol(raw_coords(1:n_raw,:), tol, ...
                                 'ByRows', true, 'DataScale', 1);

hex_nodes   = zeros(nHex, 6);
for k = 1:nHex
    for j = 1:6
        hex_nodes(k,j) = ic(raw_hex(k,j));
    end
end
inner_nodes = ic(inner_raw);

if use_hub
    hub_node = ic(hub_raw_idx);
end

nNodes = size(node_coords, 1);

switch joint_type
    case 'Y'
        exp_nodes = nHex*6 + 1;
    case 'triangle'
        exp_nodes = nHex*6;       % inner vertices coincide with triangle corners
end
fprintf('Joint type   : %s\n', joint_type);
fprintf('Unique nodes : %d  (expected %d)\n', nNodes, exp_nodes);

%% --- Bar connectivity ------------------------------------------------------
% Hexagon bars: 6 perimeter edges + 3 internal fan diagonals per hex.
% Joint bars  : 3 prongs (Y) or 3 triangle sides (triangle).

max_bars = nHex*9 + 3 + 1;    % generous upper bound
raw_bars = zeros(max_bars, 2);
nb = 0;

for k = 1:nHex
    v = hex_nodes(k,:);
    for j = 1:6
        nb = nb+1;
        raw_bars(nb,:) = [v(j), v(mod(j,6)+1)];
    end
    for j = 2:4
        nb = nb+1;
        raw_bars(nb,:) = [v(1), v(j+1)];
    end
end

if use_hub
    for k = 1:nHex
        nb = nb+1;
        raw_bars(nb,:) = [hub_node, inner_nodes(k)];
    end
else
    for k = 1:nHex
        k2 = mod(k, nHex) + 1;
        nb = nb+1;
        raw_bars(nb,:) = [inner_nodes(k), inner_nodes(k2)];
    end
end

raw_bars = sort(raw_bars(1:nb,:), 2);
bar_conn = unique(raw_bars, 'rows');
nBars    = size(bar_conn, 1);
fprintf('Unique bars  : %d\n', nBars);

%% --- Identify joint bars ---------------------------------------------------
% act_ids(k) = bar index for the joint bar associated with hex k.
% Y-joint   : bar from hub_node      → inner_nodes(k)
% Triangle  : bar from inner_nodes(k) → inner_nodes(k+1)
% This ordering ensures column k of L0_his controls the joint at hex k.

is_joint = false(nBars, 1);
act_ids  = zeros(nHex, 1);

if use_hub
    for k = 1:nHex
        bp  = sort([hub_node, inner_nodes(k)]);
        row = find(bar_conn(:,1)==bp(1) & bar_conn(:,2)==bp(2), 1);
        if ~isempty(row)
            is_joint(row) = true;
            act_ids(k)    = row;
        end
    end
else
    for k = 1:nHex
        k2  = mod(k, nHex) + 1;
        bp  = sort([inner_nodes(k), inner_nodes(k2)]);
        row = find(bar_conn(:,1)==bp(1) & bar_conn(:,2)==bp(2), 1);
        if ~isempty(row)
            is_joint(row) = true;
            act_ids(k)    = row;
        end
    end
end
fprintf('Joint bars   : %d  (expected 3)\n', sum(is_joint));

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

assembly       = Assembly_Hex_Origami;
assembly.node  = node;
assembly.bar   = bar;
assembly.Initialize_Assembly();

%% --- Support conditions ----------------------------------------------------
% Panel 3 (lower-left, clamped): X and Z fixed.
% All nodes: Y fixed (keep motion in XZ plane).

hex3_nodes = unique(hex_nodes(3,:));

supp      = zeros(nNodes, 4);
supp(:,1) = (1:nNodes)';
supp(:,3) = 1;

for n = hex3_nodes(:)'
    supp(n,2) = 1;
    supp(n,4) = 1;
end

%% --- External force --------------------------------------------------------
% Optional disturbance: outward (+X) force on outer nodes of panel 1.
% Set F_mag = 0 (above) for pure actuation mode.

outer_hex1 = setdiff(hex_nodes(1,:), inner_nodes(1));
nFN        = numel(outer_hex1);

totalSteps = round(t_total / dt);
stepForce  = round(t_force / dt);

Fext = zeros(totalSteps, nNodes, 3);
for n = outer_hex1(:)'
    Fext(1:stepForce, n, 1) = F_mag / nFN;
end

%% --- Actuation trajectory --------------------------------------------------
% Each joint bar is a linear actuator with a prescribed rest length L0(t).
% The bar generates force E*A*(L_current - L0(t))/L0_ref, driving the
% connected panels toward the target configuration.
%
% act_ids(k) : bar index for joint k  (Y: hub→hex k; triangle: side k)
% L0_nat(k)  : natural rest length of joint bar k
% L0_his     : (totalSteps × nHex) prescribed rest lengths
%
% Actuation demo sequence (bar 1 only, controlling panel 1):
%   Phase 1  0  → t_ramp          ramp bar 1 from L0 to L0*(1+strain_act)
%   Phase 2  t_ramp → t_ramp+t_hold  hold contracted
%   Phase 3  t_ramp+t_hold → ...  release back to L0
%   Remainder                      free vibration / hold natural

L0_nat = bar.L0_vec(act_ids);   % natural rest lengths (set by Initialize_Assembly)

strain_act = -0.2;             % fractional actuation (negative = contract)
t_ramp     = 2.0;               % ramp time (s)
t_hold     = 2.0;               % hold time (s)

ramp_steps = round(t_ramp / dt);
hold_steps = round(t_hold / dt);

L0_his = repmat(L0_nat', totalSteps, 1);  % (totalSteps × nHex), default = natural

for k = 1:nHex
    dL = strain_act * L0_nat(k);
    % ramp in
    for s = 1:min(ramp_steps, totalSteps)
        L0_his(s, k) = L0_nat(k) + dL * (s / ramp_steps);
    end
    % hold
    i_hold_end = min(ramp_steps + hold_steps, totalSteps);
    L0_his(ramp_steps+1 : i_hold_end, k) = L0_nat(k) + dL;
    % ramp out
    for s = 1:ramp_steps
        idx = ramp_steps + hold_steps + s;
        if idx <= totalSteps
            L0_his(idx, k) = L0_nat(k) + dL * (1 - s/ramp_steps);
        end
    end
    % actuate one bar at a time: bar k starts after bar k-1 finishes
    % shift this bar's sequence forward by (k-1) full cycles
    cycle = ramp_steps + hold_steps + ramp_steps;
    shift = (k-1) * cycle;
    L0_his(:, k) = L0_nat(k);   % reset to natural
    for s = 1:ramp_steps
        idx = shift + s;
        if idx <= totalSteps
            L0_his(idx, k) = L0_nat(k) + dL * (s / ramp_steps);
        end
    end
    for s = 1:hold_steps
        idx = shift + ramp_steps + s;
        if idx <= totalSteps
            L0_his(idx, k) = L0_nat(k) + dL;
        end
    end
    for s = 1:ramp_steps
        idx = shift + ramp_steps + hold_steps + s;
        if idx <= totalSteps
            L0_his(idx, k) = L0_nat(k) + dL * (1 - s/ramp_steps);
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
% Joint bars are colored by actuation ratio:
%   red   (ratio < 1): contracted
%   green (ratio = 1): natural length
%   blue  (ratio > 1): extended

Uhis_sub = Uhis(1:skipFr:end, :, :);
nFrames  = size(Uhis_sub, 1);

mp4File = fullfile(fileparts(mfilename('fullpath')), ...
                   sprintf('three_hex_%s.mp4', lower(joint_type)));
vid = VideoWriter(mp4File, 'MPEG-4');
vid.FrameRate = fps;
open(vid);

fig = figure('color','white','position',[0 0 800 750]);

hex_face = [1.00 0.82 0.10;   % panel 1 — gold
            0.30 0.55 0.80;   % panel 2 — steel blue
            0.25 0.25 0.25];  % panel 3 — dark (clamped)

act_cmap = [1 0.15 0.15;   % contracted → red
            0.10 0.75 0.20; % natural    → green
            0.15 0.35 1.0]; % extended   → blue
act_ratio_pts = [0.80, 1.00, 1.20];

pad  = R * 0.4;
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

    % Compute actuation ratio for each joint bar at this frame
    ratio = L0_his(step, :) ./ L0_nat';          % (1 × nHex)
    ratio_clamped = max(act_ratio_pts(1), min(act_ratio_pts(end), ratio));
    bar_clr = interp1(act_ratio_pts, act_cmap, ratio_clamped);  % (nHex × 3)

    % --- Undeformed ghost ---------------------------------------------------
    for k = 1:nHex
        vxz = node_coords(hex_nodes(k,:), [1 3]);
        patch(vxz(:,1), vxz(:,2), [0.93 0.93 0.93], ...
              'EdgeColor',[0.75 0.75 0.75], 'FaceAlpha',0.4, ...
              'LineWidth',0.5, 'LineStyle','--');
    end
    if use_hub
        for k = 1:nHex
            p1 = node_coords(hub_node,       [1 3]);
            p2 = node_coords(inner_nodes(k), [1 3]);
            plot([p1(1) p2(1)],[p1(2) p2(2)],'--', ...
                 'Color',[0.75 0.75 0.75],'LineWidth',1.0);
        end
    else
        tri_ref = node_coords(inner_nodes([1 2 3 1]), [1 3]);
        plot(tri_ref(:,1), tri_ref(:,2), '--', ...
             'Color',[0.75 0.75 0.75],'LineWidth',1.0);
    end

    % --- Deformed panels (filled) -------------------------------------------
    for k = 1:nHex
        vxz = deformNode(hex_nodes(k,:), [1 3]);
        patch(vxz(:,1), vxz(:,2), hex_face(k,:), ...
              'EdgeColor','k', 'FaceAlpha',1.0, 'LineWidth',1.8);
    end

    % --- Deformed joint bars (colored by actuation ratio) -------------------
    if use_hub
        for k = 1:nHex
            p1 = deformNode(hub_node,       [1 3]);
            p2 = deformNode(inner_nodes(k), [1 3]);
            plot([p1(1) p2(1)],[p1(2) p2(2)],'-', ...
                 'Color',bar_clr(k,:),'LineWidth',4.0);
        end
        hxz = deformNode(hub_node, [1 3]);
        plot(hxz(1), hxz(2), 'o', 'MarkerSize',10, ...
             'MarkerFaceColor',[0.2 0.2 0.2],'MarkerEdgeColor','k','LineWidth',1.2);
    else
        for k = 1:nHex
            k2 = mod(k, nHex) + 1;
            p1 = deformNode(inner_nodes(k),  [1 3]);
            p2 = deformNode(inner_nodes(k2), [1 3]);
            plot([p1(1) p2(1)],[p1(2) p2(2)],'-', ...
                 'Color',bar_clr(k,:),'LineWidth',4.0);
        end
        tri_def = deformNode(inner_nodes, [1 3]);
        patch(tri_def(:,1), tri_def(:,2), [0.8 0.8 0.8], ...
              'FaceAlpha',0.20, 'EdgeColor','none');
        plot(deformNode(inner_nodes([1 2 3 1]),[1]), ...
             deformNode(inner_nodes([1 2 3 1]),[3]), 'k-', 'LineWidth',0.5);
        for k = 1:nHex
            plot(deformNode(inner_nodes(k),1), deformNode(inner_nodes(k),3), 'o', ...
                 'MarkerSize',8,'MarkerFaceColor',bar_clr(k,:), ...
                 'MarkerEdgeColor','k','LineWidth',1.2);
        end
    end

    % --- Force arrow (if external force active) -----------------------------
    realTime = fi * skipFr * dt;
    if realTime <= t_force && F_mag > 0
        fc = mean(deformNode(outer_hex1, [1 3]), 1);
        quiver(fc(1), fc(2), R*0.5, 0, 0, 'r', ...
               'LineWidth',2.5, 'MaxHeadSize',0.8);
    end

    % --- Actuation state summary --------------------------------------------
    act_str = sprintf('bar1:%.0f%%  bar2:%.0f%%  bar3:%.0f%%', ...
                      ratio(1)*100, ratio(2)*100, ratio(3)*100);
    title(sprintf('%s joint  |  t = %.2f s\n%s', joint_type, realTime, act_str), ...
          'FontSize',11);
    drawnow;
    writeVideo(vid, getframe(fig));
end

close(vid); close(fig);
fprintf('Saved: %s\n', mp4File);

%% --- Displacement & actuation time-history ---------------------------------
time   = (0:totalSteps-1)*dt;
u_hex  = zeros(totalSteps, nHex);
for k = 1:nHex
    u_hex(:,k) = mean(squeeze(Uhis(:, hex_nodes(k,:), 1)), 2);
end

clrs_hex = hex_face;
leg_strs = {'Panel 1 (actuated)', 'Panel 2 (actuated)', 'Panel 3 (clamped)'};

figure('Position',[100 100 900 500]);

subplot(2,1,1);
hold on; grid on;
for k = 1:nHex
    plot(time, u_hex(:,k)*1e3, 'Color',clrs_hex(k,:), 'LineWidth',1.5);
end
ylabel('Centroid X-disp (mm)');
title(sprintf('Panel response — %s joint', joint_type));
legend(leg_strs, 'Location','best');

subplot(2,1,2);
hold on; grid on;
act_clrs = [1 0 0; 0 0.6 0; 0 0 1];
for k = 1:nHex
    plot(time, L0_his(:,k) ./ L0_nat(k), 'Color',act_clrs(k,:), 'LineWidth',1.5);
end
yline(1,'k--','Natural','FontSize',9);
ylabel('Rest-length ratio  L0(t)/L0_{nat}');
xlabel('Time (s)');
legend({'Bar 1','Bar 2','Bar 3'}, 'Location','best');
