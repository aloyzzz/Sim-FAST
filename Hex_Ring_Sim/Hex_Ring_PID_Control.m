%% Hex_Ring_PID_Control.m
%
% Closed-loop PID control of a hexagonal solar-panel ring (A-ring).
%
% This script extends Hex_Ring_Dynamic.m by giving the inter-panel joints
% an *active* role: each of the 6 joint bars is treated as a linear
% actuator whose eigen-(pre)strain is set every time step by a decentralised
% PID controller. The goal of the controller is to COUNTERACT an external
% disturbance force and hold the ring close to its reference (undeformed)
% shape.
%
% How the control authority is generated
% --------------------------------------
% A bar's stress is  Sx = E*(Ex - prestrain).  Commanding a negative
% prestrain therefore raises the bar tension and pulls the two connected
% panels together; a positive prestrain pushes them apart. Each joint bar
% thus behaves as a contraction/extension actuator embedded in the joint.
%
% Decentralised strain-feedback PID (one loop per joint bar)
% ----------------------------------------------------------
%   measurement : Ex_k(t)             current strain of joint bar k
%   setpoint    : 0                    (joint at its natural length)
%   error       : e_k = 0 - Ex_k
%   command     : prestrain_k = Kp*e_k + Ki*∫e_k dt + Kd*de_k/dt
%
% When the disturbance stretches a joint (Ex_k > 0) the error is negative,
% so the command prestrain is negative, which adds tension that pulls the
% joint back toward its natural length — negative feedback that rejects the
% disturbance. The integral term removes the steady-state offset that a
% sustained external force would otherwise leave behind.
%
% The script runs the SAME disturbance twice — once with the controller OFF
% (passive ring) and once with it ON — and compares the two responses.
% Outputs: hex_ring_pid_control.mp4 (side-by-side animation) and a
% time-history figure of panel deflection and actuator commands.

clear; close all; clc;

%% --- Paths -----------------------------------------------------------------
baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Solver')));

%% --- Parameters ------------------------------------------------------------
R           = 0.50;      % hexagon circumradius (m)
barA        = 1e-4;      % bar cross-section area (m^2)
E_panel     = 70e9;      % Young's modulus inside each panel (Pa) — aluminium
E_joint     = 1e8;       % joint Young's modulus (Pa) — semi-rigid compliant joint

m_node      = 10.0;      % lumped mass per node (kg)

F_mag       = 80.0;      % sustained external disturbance force (N)
t_rampF     = 0.5;       % disturbance ramp-in time (s)
t_total     = 12.0;      % total simulation time (s)
dt          = 0.01;      % time step (s)

alpha_d     = 0.10;      % Rayleigh mass-proportional damping
beta_d      = 0.0;       % Rayleigh stiffness-proportional damping

% --- PID gains (illustrative; tune for your loading/stiffness) -------------
% Loops act on bar engineering strain (dimensionless), command is prestrain
% (dimensionless eigen-strain). Gains below give a well-damped response that
% drives the panel deflection back toward zero under the sustained load.
Kp_gain     = 3.0;
Ki_gain     = 80.0;
Kd_gain     = 0.05;
prestrain_lim = 0.05;    % actuator stroke limit |prestrain| <= 5%
t_ctrl_on   = 1.0;       % controller switches on at t = 1 s (so we first see drift)

skipFr      = 5;         % frame subsampling for MP4
fps         = 20;        % MP4 playback frame rate

%% --- Hexagonal-ring geometry (identical to Hex_Ring_Dynamic) ---------------
d_ring = R * sqrt(3);

raw_coords = zeros(36, 3);
raw_hex    = zeros(6, 6);

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

tol = R * 1e-6;
[node_coords, ~, ic] = uniquetol(raw_coords, tol, 'ByRows', true, 'DataScale', 1);
hex_nodes = reshape(ic(raw_hex(:)), 6, 6);
nNodes = size(node_coords, 1);
fprintf('Unique nodes: %d  (expected 24)\n', nNodes);

%% --- Bar connectivity ------------------------------------------------------
raw_bars = zeros(6*9, 2);
nb = 0;
for k = 1:6
    v = hex_nodes(k,:);
    for j = 1:6
        nb = nb+1; raw_bars(nb,:) = [v(j), v(mod(j,6)+1)];
    end
    for j = 2:4
        nb = nb+1; raw_bars(nb,:) = [v(1), v(j+1)];
    end
end
raw_bars = sort(raw_bars(1:nb,:), 2);
bar_conn = unique(raw_bars, 'rows');
nBars    = size(bar_conn, 1);

%% --- Identify inter-panel joint bars (the actuators) -----------------------
is_joint = false(nBars, 1);
for k = 1:6
    k2 = mod(k, 6) + 1;
    shared = intersect(hex_nodes(k,:), hex_nodes(k2,:));
    if numel(shared) == 2
        bp = sort(shared(:)');
        row = find(bar_conn(:,1)==bp(1) & bar_conn(:,2)==bp(2), 1);
        if ~isempty(row), is_joint(row) = true; end
    end
end
act_ids  = find(is_joint);          % the 6 actuated joint bars
nAct     = numel(act_ids);
fprintf('Joint (actuated) bars: %d  (expected 6)\n', nAct);

%% --- Elements & assembly ---------------------------------------------------
node = Elements_Nodes;
node.coordinates_mat = node_coords;
node.mass_vec = m_node * ones(nNodes, 1);

bar = Vec_Elements_Bars;
bar.node_ij_mat = bar_conn;
bar.A_vec = barA * ones(nBars, 1);
E_vec = E_panel * ones(nBars, 1);
E_vec(is_joint) = E_joint;
bar.E_vec = E_vec;

assembly = Assembly_Hex_Origami;
assembly.node = node;
assembly.bar  = bar;
assembly.Initialize_Assembly();

%% --- Support conditions ----------------------------------------------------
% Fix panel 4 (opposite the loaded panel); constrain Y for every node.
hex4_nodes = unique(hex_nodes(4,:));
supp = zeros(nNodes, 4);
supp(:,1) = (1:nNodes)';
supp(:,3) = 1;                  % all nodes: Y fixed (out-of-plane)
for n = hex4_nodes(:)'
    supp(n,2) = 1;  supp(n,4) = 1;   % X and Z fixed
end

%% --- External disturbance force --------------------------------------------
% Sustained +X force on the outer nodes of panel 1 (ramped in over t_rampF).
shared_nbrs = unique([hex_nodes(2,:), hex_nodes(6,:)]);
outer_hex1  = setdiff(hex_nodes(1,:), shared_nbrs);
nFN         = numel(outer_hex1);

totalSteps = round(t_total / dt);
rampSteps  = round(t_rampF / dt);

ramp = ones(totalSteps,1);
ramp(1:rampSteps) = linspace(0,1,rampSteps);

Fext = zeros(totalSteps, nNodes, 3);
for n = outer_hex1(:)'
    Fext(:, n, 1) = (F_mag / nFN) * ramp;
end

%% --- Controller specification ----------------------------------------------
control.bar_ids         = act_ids;
control.Kp              = Kp_gain;
control.Ki              = Ki_gain;
control.Kd              = Kd_gain;
control.target_strain   = 0;          % regulate each joint to natural length
control.prestrain_limit = prestrain_lim;
control.t_on            = t_ctrl_on;

%% --- Run 1: passive ring (controller OFF) ----------------------------------
fprintf('\n[Run 1] Passive ring (no control)...\n');
assembly.Initialize_Assembly();        % reset displacement & prestrain
caa = Solver_CAA_Dynamics;
caa.assembly = assembly;
caa.supp = supp;  caa.dt = dt;  caa.Fext = Fext;
caa.alpha = alpha_d;  caa.beta = beta_d;
caa.rotSprTargetAngle = [];
Uhis_off = caa.Solve();
fprintf('Done.\n');

%% --- Run 2: actively controlled ring (controller ON) -----------------------
fprintf('\n[Run 2] PID-controlled ring...\n');
assembly.Initialize_Assembly();        % reset displacement & prestrain
caa = Solver_CAA_Dynamics;
caa.assembly = assembly;
caa.supp = supp;  caa.dt = dt;  caa.Fext = Fext;
caa.alpha = alpha_d;  caa.beta = beta_d;
caa.rotSprTargetAngle = [];
caa.control = control;
[Uhis_on, ctrlLog] = caa.Solve();
fprintf('Done.\n');

%% --- Side-by-side comparison MP4 -------------------------------------------
Uoff_sub = Uhis_off(1:skipFr:end, :, :);
Uon_sub  = Uhis_on (1:skipFr:end, :, :);
nFrames  = size(Uon_sub, 1);

mp4File = fullfile(fileparts(mfilename('fullpath')), 'hex_ring_pid_control.mp4');
vid = VideoWriter(mp4File, 'MPEG-4');
vid.FrameRate = fps;  open(vid);

fig = figure('color','white','position',[0 0 1300 720]);

hex_face = [1.00 0.82 0.10;   % 1 — gold (loaded)
            0.30 0.55 0.80;   % 2
            0.30 0.55 0.80;   % 3
            0.25 0.25 0.25;   % 4 — dark (clamped)
            0.30 0.55 0.80;   % 5
            0.30 0.55 0.80];  % 6

% Colormap for actuator command: contract (red) - neutral (green) - extend (blue)
act_cmap      = [1 0.15 0.15; 0.10 0.75 0.20; 0.15 0.35 1.0];
act_cmd_pts   = [-prestrain_lim, 0, prestrain_lim];

pad  = R * 0.3;
xAll = node_coords(:,1);  zAll = node_coords(:,3);
xLim = [min(xAll)-R-pad, max(xAll)+R+pad];
zLim = [min(zAll)-R-pad, max(zAll)+R+pad];

for fi = 1:nFrames
    clf;
    realTime = fi * skipFr * dt;
    step     = min((fi-1)*skipFr + 1, totalSteps);

    % cmd for this frame (controlled run only)
    cmd  = ctrlLog.prestrain_his(step, :);
    cmd_cl = max(act_cmd_pts(1), min(act_cmd_pts(end), cmd));
    bar_clr = interp1(act_cmd_pts, act_cmap, cmd_cl);

    for sp = 1:2
        subplot(1,2,sp); hold on; axis equal off;
        xlim(xLim); ylim(zLim);
        if sp==1
            U = squeeze(Uoff_sub(fi,:,:));  ttl = 'Passive  (control OFF)';
        else
            U = squeeze(Uon_sub(fi,:,:));   ttl = 'Active PID  (control ON)';
        end
        deformNode = node_coords + U;

        % undeformed ghost
        for k = 1:6
            vxz = node_coords(hex_nodes(k,:), [1 3]);
            patch(vxz(:,1), vxz(:,2), [0.93 0.93 0.93], ...
                  'EdgeColor',[0.78 0.78 0.78], 'FaceAlpha',0.35, ...
                  'LineWidth',0.5, 'LineStyle','--');
        end
        % deformed panels
        for k = 1:6
            vxz = deformNode(hex_nodes(k,:), [1 3]);
            patch(vxz(:,1), vxz(:,2), hex_face(k,:), ...
                  'EdgeColor','k', 'FaceAlpha',1.0, 'LineWidth',1.6);
        end
        % joint bars (colored by command on the active side, white otherwise)
        for b = 1:nAct
            n12 = bar_conn(act_ids(b),:);
            p1 = deformNode(n12(1),[1 3]);  p2 = deformNode(n12(2),[1 3]);
            if sp==2
                clr = bar_clr(b,:);
            else
                clr = [1 1 1];
            end
            plot([p1(1) p2(1)],[p1(2) p2(2)],'-','Color',clr,'LineWidth',3.0);
        end
        % disturbance arrow
        fc = mean(deformNode(outer_hex1,[1 3]),1);
        quiver(fc(1)+R*0.55, fc(2), -R*0.45, 0, 0, 'r', ...
               'LineWidth',2.5, 'MaxHeadSize',1.0);
        title(ttl, 'FontSize',12);
    end

    if realTime < t_ctrl_on
        phase = 'controller idle';
    else
        phase = 'controller active';
    end
    sgtitle(sprintf('Hex solar ring  |  F = %.0f N disturbance  |  t = %.2f s  (%s)', ...
                    F_mag, realTime, phase), 'FontSize',13);
    drawnow;
    writeVideo(vid, getframe(fig));
end
close(vid); close(fig);
fprintf('Saved: %s\n', mp4File);

%% --- Time-history comparison -----------------------------------------------
time     = (0:totalSteps-1)*dt;
uoff_h1  = mean(squeeze(Uhis_off(:, hex_nodes(1,:), 1)), 2);   % panel-1 X centroid
uon_h1   = mean(squeeze(Uhis_on (:, hex_nodes(1,:), 1)), 2);

figure('Position',[100 100 980 640]);

subplot(2,1,1); hold on; grid on;
plot(time, uoff_h1*1e3, 'Color',[0.75 0.20 0.20], 'LineWidth',1.8, ...
     'DisplayName','Passive (no control)');
plot(time, uon_h1 *1e3, 'Color',[0.15 0.45 0.85], 'LineWidth',1.8, ...
     'DisplayName','Active PID control');
xline(t_ctrl_on,'k--','controller on','LabelVerticalAlignment','bottom','FontSize',9);
yline(0,'k:');
ylabel('Panel 1 centroid X-disp (mm)');
title('Disturbance rejection: panel 1 deflection under sustained external force');
legend('Location','best');

subplot(2,1,2); hold on; grid on;
act_c = lines(nAct);
for b = 1:nAct
    plot(time, ctrlLog.prestrain_his(:,b)*100, 'Color',act_c(b,:), 'LineWidth',1.1, ...
         'DisplayName',sprintf('joint %d',b));
end
yline( prestrain_lim*100,'k--','stroke limit','FontSize',8);
yline(-prestrain_lim*100,'k--');
ylabel('Actuator command  prestrain (%)');
xlabel('Time (s)');
title('PID actuator commands (eigen-strain) for the 6 joint actuators');
legend('Location','best','FontSize',8,'NumColumns',2);

% --- Console summary --------------------------------------------------------
ss_off = uoff_h1(end)*1e3;
ss_on  = uon_h1(end)*1e3;
fprintf('\n--- Steady-state panel-1 X-deflection ---\n');
fprintf('  Passive    : %+7.3f mm\n', ss_off);
fprintf('  PID control: %+7.3f mm\n', ss_on);
if abs(ss_off) > 1e-9
    fprintf('  Reduction  : %5.1f %%\n', 100*(1-abs(ss_on)/abs(ss_off)));
end
