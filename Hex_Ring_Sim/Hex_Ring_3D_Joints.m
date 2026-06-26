%% Hex_Ring_3D_Joints.m
%
% 3-D response of the JOINTS in a hexagonal solar-panel array.
%
% This is NOT a hinge model. Each panel is a stiff triangulated hexagon and
% the panel-to-panel connections are modelled as compliant *bar joints*
% (exactly as in Hex_Center_Ring.m). The purpose here is to let the array
% move in full 3-D — the out-of-plane (Y) degree of freedom is left free —
% and to study what an external force does to those joints: how much each
% joint stretches and how much axial force it carries.
%
% Array  : 1 center hexagon + 6 ring hexagons.
% Joints : bar connectors across each gap
%            6 center-ring + 6 ring-ring  (12 joint bars total).
%          'triangle' -> one bar per gap;  'Y' -> hub node + 2 prong bars.
% Loading: an OUT-OF-PLANE (+Y) impulse force on ring panel 1, applied for
%          t_force seconds then released, so we see the joints flex out of
%          plane and the array oscillate and settle in 3-D.
% Support: ring panel 4 (opposite the load) is fully clamped (X,Y,Z) — this
%          also removes all rigid-body modes, so nothing else is constrained
%          and the structure is free to deform in 3-D everywhere else.
%
% Optional: set use_control = true to also engage the decentralised joint
%           PID controller (Solver_CAA_Dynamics.control) and compare how the
%           active joints reject the same disturbance in 3-D.
%
% Outputs : hex_ring_3d_joints.mp4  (3-D animation, joints coloured by force)
%           + joint elongation / joint force time-history figures.

clear; close all; clc;

%% --- Options ---------------------------------------------------------------
joint_type  = 'triangle';   % 'triangle' (1 bar/gap) or 'Y' (hub + 2 prongs)
use_control = false;        % true -> also run the active-joint PID case

%% --- Paths -----------------------------------------------------------------
baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Solver')));

%% --- Parameters ------------------------------------------------------------
R           = 0.50;      % hexagon circumradius (m)
barA        = 1e-4;      % bar cross-section area (m^2)
E_panel     = 70e9;      % panel Young's modulus (Pa) — aluminium
E_joint     = 1e8;       % joint Young's modulus (Pa) — compliant connector

m_node      = 10.0;      % lumped mass per node (kg)

F_mag       = 150.0;     % OUT-OF-PLANE (+Y) impulse force (N)
t_force     = 1.0;       % force-on duration (s)
t_total     = 8.0;       % total simulation time (s)
dt          = 0.01;      % time step (s)

alpha_d     = 0.20;      % Rayleigh mass-proportional damping (settles motion)
beta_d      = 0.0;

% PID gains used only if use_control = true (act on joint-bar strain)
Kp_gain = 3.0;  Ki_gain = 80.0;  Kd_gain = 0.05;  prestrain_lim = 0.05;

skipFr      = 4;
fps         = 20;

%% --- Geometry (1 center + 6 ring hexes, with gap joints) -------------------
nRing  = 6;
nHex   = 1 + nRing;
d_ring = 3 * R;
use_hub = strcmp(joint_type, 'Y');

n_panel_raw = nHex * 6;
max_raw     = n_panel_raw + 12 + 5;
raw_coords  = zeros(max_raw, 3);
raw_hex     = zeros(nHex, 6);

% Center hex (h=1) in the XZ plane (Y = 0)
for j = 1:6
    raw_coords(j, :) = [R*cos((j-1)*pi/3), 0, R*sin((j-1)*pi/3)];
    raw_hex(1, j) = j;
end
% Ring hexes
for k = 1:nRing
    theta_k = (k-1) * pi/3;
    cx = d_ring * cos(theta_k);  cz = d_ring * sin(theta_k);
    for j = 1:6
        phi_j = (theta_k + pi) + (j-1)*pi/3;
        idx   = 6 + (k-1)*6 + j;
        raw_coords(idx, :) = [cx + R*cos(phi_j), 0, cz + R*sin(phi_j)];
        raw_hex(k+1, j) = idx;
    end
end
n_placed = n_panel_raw;

cr_hub_raw = zeros(nRing, 1);  rr_hub_raw = zeros(nRing, 1);
if use_hub
    for k = 1:nRing
        theta_k = (k-1)*pi/3;
        n_placed = n_placed + 1;  cr_hub_raw(k) = n_placed;
        raw_coords(n_placed, :) = [1.5*R*cos(theta_k), 0, 1.5*R*sin(theta_k)];
    end
    for k = 1:nRing
        k2 = mod(k, nRing) + 1;
        p1 = raw_coords(raw_hex(k+1, 6), :);
        p2 = raw_coords(raw_hex(k2+1, 2), :);
        n_placed = n_placed + 1;  rr_hub_raw(k) = n_placed;
        raw_coords(n_placed, :) = (p1 + p2) / 2;
    end
end
n_raw = n_placed;

tol = R * 1e-6;
[node_coords, ~, ic] = uniquetol(raw_coords(1:n_raw, :), tol, ...
                                 'ByRows', true, 'DataScale', 1);
hex_nodes = zeros(nHex, 6);
for h = 1:nHex
    for j = 1:6, hex_nodes(h, j) = ic(raw_hex(h, j)); end
end
center_nodes     = hex_nodes(1, :);
inner_ring_nodes = hex_nodes(2:end, 1);
if use_hub, cr_hub = ic(cr_hub_raw);  rr_hub = ic(rr_hub_raw); end
nNodes = size(node_coords, 1);
fprintf('Joint type   : %s\n', joint_type);
fprintf('Unique nodes : %d\n', nNodes);

%% --- Bar connectivity ------------------------------------------------------
max_bars = nHex*9 + 50;
raw_bars = zeros(max_bars, 2);
is_joint_flag = false(max_bars, 1);
nb = 0;
for h = 1:nHex
    v = hex_nodes(h, :);
    for j = 1:6, nb = nb+1; raw_bars(nb,:) = [v(j), v(mod(j,6)+1)]; end
    for j = 2:4, nb = nb+1; raw_bars(nb,:) = [v(1), v(j+1)]; end
end
if use_hub
    for k = 1:nRing
        nb=nb+1; raw_bars(nb,:)=[center_nodes(k), cr_hub(k)];        is_joint_flag(nb)=true;
        nb=nb+1; raw_bars(nb,:)=[cr_hub(k), inner_ring_nodes(k)];    is_joint_flag(nb)=true;
    end
    for k = 1:nRing
        k2 = mod(k, nRing) + 1;
        nb=nb+1; raw_bars(nb,:)=[hex_nodes(k+1,6), rr_hub(k)];       is_joint_flag(nb)=true;
        nb=nb+1; raw_bars(nb,:)=[rr_hub(k), hex_nodes(k2+1,2)];      is_joint_flag(nb)=true;
    end
else
    for k = 1:nRing
        nb=nb+1; raw_bars(nb,:)=[center_nodes(k), inner_ring_nodes(k)]; is_joint_flag(nb)=true;
    end
    for k = 1:nRing
        k2 = mod(k, nRing) + 1;
        nb=nb+1; raw_bars(nb,:)=[hex_nodes(k+1,6), hex_nodes(k2+1,2)]; is_joint_flag(nb)=true;
    end
end
raw_bars_s = sort(raw_bars(1:nb,:), 2);
flag_s     = is_joint_flag(1:nb);
[bar_conn, ~, ic_bar] = unique(raw_bars_s, 'rows');
nBars = size(bar_conn, 1);
is_joint = false(nBars, 1);
for b = 1:nb
    if flag_s(b), is_joint(ic_bar(b)) = true; end
end
act_ids  = find(is_joint);
nJoint   = numel(act_ids);
fprintf('Unique bars  : %d  (joint bars: %d)\n', nBars, nJoint);

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

L0_joint = bar.L0_vec(act_ids);     % natural joint lengths (for elongation %)

%% --- Support conditions (3-D: Y is FREE) -----------------------------------
% Only ring hex 4 is clamped (X,Y,Z). Everything else is free in all 3 axes.
ring4_nodes = unique(hex_nodes(5, :));   % ring hex 4 = h=5
supp = zeros(numel(ring4_nodes), 4);
supp(:,1) = ring4_nodes(:);
supp(:,2) = 1;  supp(:,3) = 1;  supp(:,4) = 1;   % X, Y, Z all fixed
if use_hub
    extra = [cr_hub(4); rr_hub(3); rr_hub(4)];
    supp = [supp; [extra, ones(numel(extra),3)]];
end

%% --- External OUT-OF-PLANE force -------------------------------------------
% +Y impulse on the outer nodes of ring panel 1 (h=2).
outer_ring1 = setdiff(hex_nodes(2,:), inner_ring_nodes(1));
nFN         = numel(outer_ring1);

totalSteps = round(t_total / dt);
stepForce  = round(t_force / dt);

Fext = zeros(totalSteps, nNodes, 3);
for n = outer_ring1(:)'
    Fext(1:stepForce, n, 2) = F_mag / nFN;   % component 2 = Y (out of plane)
end

%% --- Helper to run one simulation ------------------------------------------
runSim = @(ctrl) local_run(assembly, supp, dt, Fext, alpha_d, beta_d, ctrl);

%% --- Run: passive joints (force effect only) -------------------------------
fprintf('\n[Run] Passive joints, 3-D out-of-plane load...\n');
Uhis = runSim([]);
fprintf('Done.\n');

% Per-step joint strain / elongation / axial force (model-consistent)
[jStrain, jElong, jForce] = local_joint_history(assembly, Uhis, act_ids, ...
                                                E_joint, barA, L0_joint);

%% --- Optional: active joints (PID) -----------------------------------------
if use_control
    control.bar_ids = act_ids;  control.Kp = Kp_gain;  control.Ki = Ki_gain;
    control.Kd = Kd_gain;  control.target_strain = 0;  control.prestrain_limit = prestrain_lim;
    control.t_on = t_force;     % let the impulse hit, then actively restore
    fprintf('\n[Run] Active joints (PID), 3-D...\n');
    Uhis_c = runSim(control);
    [~, jElong_c, jForce_c] = local_joint_history(assembly, Uhis_c, act_ids, ...
                                                  E_joint, barA, L0_joint);
    fprintf('Done.\n');
end

%% --- 3-D animation ---------------------------------------------------------
Uhis_sub = Uhis(1:skipFr:end, :, :);
nFrames  = size(Uhis_sub, 1);

mp4File = fullfile(fileparts(mfilename('fullpath')), 'hex_ring_3d_joints.mp4');
vid = VideoWriter(mp4File, 'MPEG-4');  vid.FrameRate = fps;  open(vid);

fig = figure('color','white','position',[0 0 950 850]);

hex_face        = repmat([0.30 0.55 0.80], nHex, 1);
hex_face(1,:)   = [0.10 0.70 0.65];   % center — teal
hex_face(2,:)   = [1.00 0.82 0.10];   % ring 1 — gold (loaded)
hex_face(5,:)   = [0.25 0.25 0.25];   % ring 4 — dark (clamped)

% Joint force colour scale (symmetric about 0)
Fscale = max(abs(jForce(:)));  if Fscale < eps, Fscale = 1; end
jcmap  = [0.15 0.35 1.0; 0.85 0.85 0.85; 1 0.15 0.15];   % compress-neutral-tension
jpts   = [-Fscale, 0, Fscale];

pad  = R * 0.6;
xAll = node_coords(:,1);  zAll = node_coords(:,3);
xLim = [min(xAll)-pad, max(xAll)+pad];
zLim = [min(zAll)-pad, max(zAll)+pad];
yMax = max(abs(Uhis(:,:,2)), [], 'all') + R*0.2;
yLim = [-yMax, yMax];

for fi = 1:nFrames
    clf; hold on; grid on; box on;
    axis equal;  xlim(xLim); ylim(yLim); zlim(zLim);
    view(35, 22);  set(gcf,'color','white');
    xlabel('X (m)'); ylabel('Y — out of plane (m)'); zlabel('Z (m)');

    step       = min((fi-1)*skipFr + 1, totalSteps);
    deformNode = node_coords + squeeze(Uhis_sub(fi,:,:));

    % reference (undeformed) ghost outlines
    for h = 1:nHex
        v = node_coords(hex_nodes(h,:), :);
        patch('XData',v(:,1),'YData',v(:,2),'ZData',v(:,3), ...
              'FaceColor',[0.93 0.93 0.93],'FaceAlpha',0.12, ...
              'EdgeColor',[0.8 0.8 0.8],'LineStyle','--','LineWidth',0.4);
    end
    % deformed panels
    for h = 1:nHex
        v = deformNode(hex_nodes(h,:), :);
        patch('XData',v(:,1),'YData',v(:,2),'ZData',v(:,3), ...
              'FaceColor',hex_face(h,:),'FaceAlpha',0.95, ...
              'EdgeColor','k','LineWidth',1.2);
    end
    % joint bars coloured by current axial force
    fcl = interp1(jpts, jcmap, max(jpts(1), min(jpts(end), jForce(step,:))));
    for b = 1:nJoint
        n12 = bar_conn(act_ids(b), :);
        p1 = deformNode(n12(1),:);  p2 = deformNode(n12(2),:);
        plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)], ...
              '-','Color',fcl(b,:),'LineWidth',4.0);
    end
    % force arrow (out of plane) while active
    realTime = fi * skipFr * dt;
    if realTime <= t_force
        fc = mean(deformNode(outer_ring1,:),1);
        quiver3(fc(1), fc(2), fc(3), 0, R*0.8, 0, 0, 'r', ...
                'LineWidth',2.5,'MaxHeadSize',2.0);
    end
    title(sprintf('3-D joint response  |  %s joints  |  t = %.2f s', ...
                  joint_type, realTime), 'FontSize',12);
    drawnow;  writeVideo(vid, getframe(fig));
end
close(vid); close(fig);
fprintf('Saved: %s\n', mp4File);

%% --- Joint diagnostics figure ----------------------------------------------
time = (0:totalSteps-1)*dt;
jc   = lines(nJoint);

figure('Position',[80 80 1000 720]);

subplot(2,1,1); hold on; grid on;
for b = 1:nJoint
    plot(time, jElong(:,b)*1e3, 'Color',jc(b,:), 'LineWidth',1.0);
end
xline(t_force,'k--','force off','LabelVerticalAlignment','bottom','FontSize',9);
ylabel('Joint elongation  \DeltaL (mm)');
title(sprintf('Effect of an out-of-plane force on the joints (%s, 3-D)', joint_type));

subplot(2,1,2); hold on; grid on;
for b = 1:nJoint
    plot(time, jForce(:,b), 'Color',jc(b,:), 'LineWidth',1.0);
end
xline(t_force,'k--','force off','LabelVerticalAlignment','bottom','FontSize',9);
yline(0,'k:');
ylabel('Joint axial force (N)');
xlabel('Time (s)');
title('Axial force carried by each joint bar');

% Out-of-plane tip deflection of the loaded panel
tipY = mean(squeeze(Uhis(:, outer_ring1, 2)), 2);

figure('Position',[120 120 900 420]); hold on; grid on;
plot(time, tipY*1e3, 'Color',[0.75 0.20 0.20], 'LineWidth',1.8, ...
     'DisplayName','Passive joints');
if use_control
    tipY_c = mean(squeeze(Uhis_c(:, outer_ring1, 2)), 2);
    plot(time, tipY_c*1e3, 'Color',[0.15 0.45 0.85], 'LineWidth',1.8, ...
         'DisplayName','Active PID joints');
    legend('Location','best');
end
xline(t_force,'k--','force off','LabelVerticalAlignment','bottom','FontSize',9);
yline(0,'k:');
ylabel('Loaded-panel out-of-plane (Y) deflection (mm)');
xlabel('Time (s)');
title('Out-of-plane deflection of the loaded panel');

%% --- Console summary -------------------------------------------------------
[peakF, idxF] = max(abs(jForce(:)));
[~, colF] = ind2sub(size(jForce), idxF);
fprintf('\n--- Joint load summary (passive) ---\n');
fprintf('  Peak |joint axial force| : %.2f N  (joint %d)\n', peakF, colF);
fprintf('  Peak |joint elongation|  : %.3f mm\n', max(abs(jElong(:)))*1e3);
fprintf('  Peak panel out-of-plane  : %.2f mm\n', max(abs(tipY))*1e3);
if use_control
    fprintf('  Peak |joint force| (PID) : %.2f N\n', max(abs(jForce_c(:))));
end

%% =========================================================================
function Uhis = local_run(assembly, supp, dt, Fext, alpha_d, beta_d, ctrl)
    assembly.Initialize_Assembly();      % reset displacement & prestrain
    caa = Solver_CAA_Dynamics;
    caa.assembly = assembly;  caa.supp = supp;  caa.dt = dt;  caa.Fext = Fext;
    caa.alpha = alpha_d;  caa.beta = beta_d;  caa.rotSprTargetAngle = [];
    if ~isempty(ctrl), caa.control = ctrl; end
    Uhis = caa.Solve();
end

function [jStrain, jElong, jForce] = local_joint_history(assembly, Uhis, ...
                                              act_ids, E_joint, barA, L0_joint)
    % Model-consistent joint strain (same Solve_Strain the solver uses),
    % elongation dL = strain*L0, axial force = E*A*strain.
    nStep   = size(Uhis,1);
    nJoint  = numel(act_ids);
    jStrain = zeros(nStep, nJoint);
    for i = 1:nStep
        Ex = assembly.bar.Solve_Strain(assembly.node, squeeze(Uhis(i,:,:)));
        jStrain(i,:) = Ex(act_ids)';
    end
    jElong = jStrain .* L0_joint';
    jForce = E_joint * barA * jStrain;
end
