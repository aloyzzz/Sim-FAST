%% Hex_Ring_SolarPanel_DispPID.m
%
% Full-3D solar-panel hex-ring simulation with displacement-based PID control.
%
% What this script demonstrates
% ------------------------------
% 1. SOLAR PANEL MODELING: Hexagonal panels are modelled with both bar
%    elements (axial stiffness) AND 4-node rotational springs at every
%    interior triangulation edge (bending stiffness, flexural rigidity D).
%    Unlike the existing hex-ring simulations that constrain Y to enforce
%    2-D behaviour, here all three DOFs are free — panels can deflect
%    out-of-plane exactly like real thin-plate solar panels.
%
% 2. DISPLACEMENT-BASED PID: An out-of-plane impulse (Y direction) is
%    applied to the outer nodes of panel 1. The controller measures the
%    Y-displacement of the panel-1 centroid nodes (sensor) and commands
%    prestrain in the joint bars connecting panel 1 to its neighbours
%    (actuators). Tensioning those joints increases geometric stiffness and
%    damps out-of-plane deflection — negative feedback on shape.
%
% Why displacement feedback (vs. the existing strain feedback)?
%    Strain PID measures deformation *in the joint bar itself*. When an
%    out-of-plane force bends the panel's interior, the joint bars may not
%    elongate significantly (they are roughly in the XZ plane). Displacement
%    feedback at the panel centroid directly detects the shape error and
%    responds regardless of the load path.
%
% The script runs three cases side-by-side:
%   Case 1: Passive (no control)
%   Case 2: Strain-based PID (existing approach, for comparison)
%   Case 3: Displacement-based PID (new Solver_CAA_Dynamics_DispPID)
%
% Outputs: hex_ring_solar_panel_disp_pid.mp4  +  time-history figure.

clear; close all; clc;

%% --- Paths -----------------------------------------------------------------
baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir,'00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir,'00_SourceCode_Solver')));

%% --- Parameters ------------------------------------------------------------
R         = 0.50;     % hexagon circumradius (m)
barA      = 1e-4;     % bar cross-section area (m^2)
E_panel   = 70e9;     % panel Young's modulus (Pa) — aluminium
E_joint   = 1e8;      % joint stiffness (Pa) — semi-rigid compliant joint
t_panel   = 3e-3;     % panel thickness (m) — 3 mm aluminium sheet
nu_panel  = 0.33;     % Poisson's ratio

m_node    = 10.0;     % lumped nodal mass (kg)
F_mag     = 150.0;    % peak out-of-plane impulse (N)
t_force   = 1.0;      % impulse duration (s)
t_total   = 6.0;      % simulation duration (s)
dt        = 0.002;    % time step (s)

alpha_d   = 1.00;     % Rayleigh mass-proportional damping
beta_d    = 0.001;    % Rayleigh stiffness-proportional damping

% --- Strain PID gains -------------------------------------------------------
% Note: for out-of-plane (Y) loading, joint bars (in XZ plane) develop only
% second-order strain (O(delta^2/L^2)), so the strain PID has very limited
% authority here. It is included for comparison purposes.
Kp_s = 3.0;   Ki_s = 30.0;   Kd_s = 0.05;
pLim_s = 0.05;   t_on_s = 1.0;

% --- Displacement PID gains (tuned for Y-displacement in metres) ------------
% Peak Y-deflection with fixed inner nodes is ~30-50 mm (cantilever plate).
% Kp: peak_deflection * Kp = pLim  ->  Kp ~ 0.05/0.04 ~ 1.5
% Ki: integrate over ~1 vibration period
% Kd: add derivative damping
Kp_d = 1.5;   Ki_d = 8.0;   Kd_d = 0.20;
pLim_d = 0.05;   t_on_d = 1.0;

skipFr = 5;    fps = 20;

%% --- Build model -----------------------------------------------------------
fprintf('Building solar-panel hex-ring model...\n');
opts.R          = R;
opts.barA       = barA;
opts.E_panel    = E_panel;
opts.E_joint    = E_joint;
opts.t_panel    = t_panel;
opts.nu_panel   = nu_panel;
opts.m_node     = m_node;
opts.joint_type = 'triangle';

mdl = Build_SolarPanel_Hex_Model(opts);

node_coords = mdl.node_coords;
hex_nodes   = mdl.hex_nodes;
bar_conn    = mdl.bar_conn;
nNodes      = size(node_coords,1);
nBars       = size(bar_conn,1);

%% --- Support conditions ----------------------------------------------------
% Ring panel 4 (hex_nodes row 5, opposite the loaded panel) is fully clamped
% in all three DOFs.  This is the only constraint needed — identical to
% Hex_Ring_3D_Joints.m which runs stably in full 3-D.
%
% The bars lie in the XZ plane (zero first-order Y stiffness at rest), but
% Kadjust = K + 2/dt*C + (4/dt²)*M adds 400 000 N/m to every diagonal entry
% via the mass term, making the system well-conditioned without extra Y pins.
ring4_nodes = mdl.ring4_nodes;
supp = zeros(numel(ring4_nodes), 4);
supp(:,1) = ring4_nodes(:);
supp(:,2) = 1;  supp(:,3) = 1;  supp(:,4) = 1;

%% --- External disturbance: out-of-plane impulse on panel 1 ----------------
outer_hex1  = mdl.outer_ring1;
nFN         = numel(outer_hex1);
totalSteps  = round(t_total / dt);
forceSteps  = round(t_force / dt);

Fext = zeros(totalSteps, nNodes, 3);
for n = outer_hex1(:)'
    Fext(1:forceSteps, n, 2) = F_mag / nFN;   % Y direction (out-of-plane)
end

%% --- Identify actuated joint bars (connecting panel 1 to neighbours) ------
% Panel 1 is ring hex 1 = hex_nodes row 2.
% Its neighbours are ring hex 2 (row 3) and ring hex 6 (row 7) plus the
% center panel (row 1). We collect all joint bars that touch any node of
% panel 1.
panel1_nodes = hex_nodes(2,:);
act_ids = [];
for b = 1:nBars
    if mdl.is_joint(b)
        n1 = bar_conn(b,1);  n2 = bar_conn(b,2);
        if any(panel1_nodes == n1) || any(panel1_nodes == n2)
            act_ids(end+1) = b; %#ok<AGROW>
        end
    end
end
act_ids = act_ids(:);
nAct = numel(act_ids);
fprintf('Actuated joint bars (panel 1 connections): %d\n', nAct);

%% --- Sensor nodes: centroid ring of panel 1 --------------------------------
% Use all nodes of panel 1 as sensors. DOF 2 = Y (out-of-plane).
sensor_nodes = panel1_nodes(:);
sensor_dofs  = 2;   % Y direction

%% --- Case 1: Passive -------------------------------------------------------
fprintf('\n[Case 1] Passive ring...\n');
mdl.assembly.Initialize_Assembly();
caa = Solver_CAA_Dynamics_DispPID;
caa.assembly = mdl.assembly;
caa.supp  = supp;  caa.dt = dt;  caa.Fext = Fext;
caa.alpha = alpha_d;  caa.beta = beta_d;
Uhis_pass = caa.Solve();
fprintf('Done.\n');

%% --- Case 2: Strain-based PID (existing approach) -------------------------
fprintf('\n[Case 2] Strain-based PID...\n');
mdl.assembly.Initialize_Assembly();
ctrl_s.bar_ids         = act_ids;
ctrl_s.Kp              = Kp_s;
ctrl_s.Ki              = Ki_s;
ctrl_s.Kd              = Kd_s;
ctrl_s.target_strain   = 0;
ctrl_s.prestrain_limit = pLim_s;
ctrl_s.t_on            = t_on_s;
caa2 = Solver_CAA_Dynamics_DispPID;
caa2.assembly = mdl.assembly;
caa2.supp = supp;  caa2.dt = dt;  caa2.Fext = Fext;
caa2.alpha = alpha_d;  caa2.beta = beta_d;
caa2.control = ctrl_s;
[Uhis_strain, log_strain] = caa2.Solve();
fprintf('Done.\n');

%% --- Case 3: Displacement-based PID (new) ----------------------------------
fprintf('\n[Case 3] Displacement-based PID...\n');
mdl.assembly.Initialize_Assembly();
ctrl_d.sensor_node_ids  = sensor_nodes;
ctrl_d.sensor_dofs      = sensor_dofs;
ctrl_d.actuator_bar_ids = act_ids;
ctrl_d.B_alloc          = [];   % auto-build from geometry
ctrl_d.Kp               = Kp_d;
ctrl_d.Ki               = Ki_d;
ctrl_d.Kd               = Kd_d;
ctrl_d.U_ref            = [];   % default: hold undeformed shape (zero disp)
ctrl_d.prestrain_limit  = pLim_d;
ctrl_d.actuator_mode    = 'bidirectional';
ctrl_d.t_on             = t_on_d;
caa3 = Solver_CAA_Dynamics_DispPID;
caa3.assembly    = mdl.assembly;
caa3.supp = supp;  caa3.dt = dt;  caa3.Fext = Fext;
caa3.alpha = alpha_d;  caa3.beta = beta_d;
caa3.disp_control = ctrl_d;
[Uhis_disp, log_disp] = caa3.Solve();
fprintf('Done.\n');

%% --- Time-history figure ---------------------------------------------------
time = (0:totalSteps-1) * dt;

% Y-displacement of panel-1 centroid across all three cases
p1_y_pass   = mean(squeeze(Uhis_pass  (:, panel1_nodes, 2)), 2);
p1_y_strain = mean(squeeze(Uhis_strain(:, panel1_nodes, 2)), 2);
p1_y_disp   = mean(squeeze(Uhis_disp  (:, panel1_nodes, 2)), 2);

figure('Position',[80 80 1050 700]);

subplot(2,1,1); hold on; grid on;
plot(time, p1_y_pass  *1e3,'Color',[0.75 0.20 0.20],'LineWidth',1.8,'DisplayName','Passive');
plot(time, p1_y_strain*1e3,'Color',[0.90 0.60 0.00],'LineWidth',1.8,'DisplayName','Strain PID (existing)');
plot(time, p1_y_disp  *1e3,'Color',[0.15 0.45 0.85],'LineWidth',1.8,'DisplayName','Displacement PID (new)');
xline(t_on_d,'k--','controller on','LabelVerticalAlignment','bottom','FontSize',9);
yline(0,'k:');
ylabel('Panel 1 centroid Y-disp (mm)');
title(['Out-of-plane disturbance rejection — solar panel hex ring  '...
       '(3-D: Y unconstrained, panel bending springs active)']);
legend('Location','best');

subplot(2,1,2); hold on; grid on;
if ~isempty(log_disp)
    act_c = lines(nAct);
    for b = 1:nAct
        plot(time, log_disp.disp.prestrain_his(:,b)*100, ...
             'Color',act_c(b,:),'LineWidth',1.1, ...
             'DisplayName',sprintf('act %d (disp PID)',b));
    end
end
yline( pLim_d*100,'k--','stroke limit','FontSize',8);
yline(-pLim_d*100,'k--');
ylabel('Actuator command — prestrain (%)');
xlabel('Time (s)');
title('Displacement-PID actuator commands (joint bars connecting panel 1)');
legend('Location','best','FontSize',8,'NumColumns',2);

% Console summary
ss = @(v) v(end)*1e3;
fprintf('\n--- Steady-state panel-1 Y-deflection ---\n');
fprintf('  Passive            : %+7.3f mm\n', ss(p1_y_pass));
fprintf('  Strain PID         : %+7.3f mm\n', ss(p1_y_strain));
fprintf('  Displacement PID   : %+7.3f mm\n', ss(p1_y_disp));
if abs(ss(p1_y_pass)) > 1e-9
    fprintf('  Strain PID reduction   : %5.1f %%\n', ...
            100*(1-abs(ss(p1_y_strain))/abs(ss(p1_y_pass))));
    fprintf('  Disp PID reduction     : %5.1f %%\n', ...
            100*(1-abs(ss(p1_y_disp))/abs(ss(p1_y_pass))));
end

%% --- MP4 animation (top-view XZ + front-view XY) --------------------------
Upass_sub   = Uhis_pass  (1:skipFr:end,:,:);
Ustrain_sub = Uhis_strain(1:skipFr:end,:,:);
Udisp_sub   = Uhis_disp  (1:skipFr:end,:,:);
nFrames     = size(Udisp_sub,1);

mp4File = fullfile(fileparts(mfilename('fullpath')), ...
                   'hex_ring_solar_panel_disp_pid.mp4');
vid = VideoWriter(mp4File,'MPEG-4');
vid.FrameRate = fps;  open(vid);

% hex_nodes row order: 1 = center, 2..7 = ring panels 1..6
% Ring panel 1 (row 2) is loaded → gold; ring panel 4 (row 5) is clamped → dark.
nHex     = mdl.nHex;   % 7 (1 center + 6 ring)
hex_face = [0.30 0.70 0.65;   % center — teal
            1.00 0.82 0.10;   % ring 1 — gold (loaded)
            0.30 0.55 0.80;   % ring 2
            0.30 0.55 0.80;   % ring 3
            0.25 0.25 0.25;   % ring 4 — dark (clamped)
            0.30 0.55 0.80;   % ring 5
            0.30 0.55 0.80];  % ring 6

xAll = node_coords(:,1);  yAll = node_coords(:,2);  zAll = node_coords(:,3); %#ok<NASGU>
pad  = R*0.3;
xLim = [min(xAll)-R-pad, max(xAll)+R+pad];
zLim = [min(zAll)-R-pad, max(zAll)+R+pad];
yLim = [-R*0.6, R*0.6];

fig = figure('color','white','Position',[0 0 1500 600]);

for fi = 1:nFrames
    clf;
    realTime = fi*skipFr*dt;
    labels = {'Passive','Strain PID','Disp PID'};
    Usubs  = {squeeze(Upass_sub(fi,:,:)), ...
              squeeze(Ustrain_sub(fi,:,:)), ...
              squeeze(Udisp_sub(fi,:,:))};

    for sp = 1:3
        U = Usubs{sp};
        deformNode = node_coords + U;

        subplot(1,3,sp); hold on; axis equal off;
        xlim(xLim); ylim(zLim);
        title(labels{sp},'FontSize',11);

        % Undeformed ghost (top-view XZ)
        for k = 1:nHex
            vxz = node_coords(hex_nodes(k,:),[1 3]);
            patch(vxz(:,1),vxz(:,2),[0.93 0.93 0.93], ...
                  'EdgeColor',[0.78 0.78 0.78],'FaceAlpha',0.3, ...
                  'LineStyle','--','LineWidth',0.5);
        end

        % Deformed panels (top-view XZ; Y deformation shown via alpha tinting)
        for k = 1:nHex
            vxz = deformNode(hex_nodes(k,:),[1 3]);
            % Tint darker for larger out-of-plane deflection
            yDef = mean(abs(deformNode(hex_nodes(k,:),2)));
            alpha_val = max(0.4, 1 - yDef/(R*0.2));
            patch(vxz(:,1),vxz(:,2),hex_face(k,:), ...
                  'EdgeColor','k','FaceAlpha',alpha_val,'LineWidth',1.5);
        end

        % Joint bars
        for b = 1:nBars
            if mdl.is_joint(b)
                n12 = bar_conn(b,:);
                p1 = deformNode(n12(1),[1 3]);
                p2 = deformNode(n12(2),[1 3]);
                isAct = any(act_ids == b);
                lw = 2.5*isAct + 0.8*(~isAct);
                clr = [0.2 0.7 0.3]*isAct + [0.7 0.7 0.7]*(~isAct);
                plot([p1(1) p2(1)],[p1(2) p2(2)],'-', ...
                     'Color',clr,'LineWidth',lw);
            end
        end

        % Disturbance arrow
        fc = mean(deformNode(outer_hex1,[1 3]),1);
        quiver(fc(1)-R*0.5, fc(2), R*0.4, 0, 0, 'Color',[0.8 0 0], ...
               'LineWidth',2,'MaxHeadSize',0.8);
    end

    phase = 'idle';
    if realTime >= t_on_d, phase = 'ctrl active'; end
    sgtitle(sprintf('Solar panel hex ring — 3D with bending springs  |  t = %.2f s  (%s)', ...
                    realTime, phase),'FontSize',12);
    drawnow;
    writeVideo(vid, getframe(fig));
end
close(vid);  close(fig);
fprintf('\nSaved: %s\n', mp4File);
