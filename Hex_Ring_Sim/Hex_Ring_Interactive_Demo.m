%% Hex_Ring_Interactive_Demo.m
%
% Runs a 3-D hex-ring joint simulation and opens an INTERACTIVE viewer so you
% can orbit / pan / zoom around the structure and scrub through time to see
% how the joints deform under load — from any angle.
%
% Pipeline:
%   1. Build_Hex_Ring_Model       -> 1 center + 6 ring hexes, compliant bar joints
%   2. Solver_CAA_Dynamics        -> 3-D dynamic response to an out-of-plane force
%      (optional decentralised joint-PID via Solver_CAA_Dynamics.control)
%   3. Interactive_Deformation_Viewer -> orbit/pan/zoom + time slider + play
%
% This is the interactive counterpart to Hex_Ring_3D_Joints.m (which renders
% an MP4). Same physics; here you drive the camera and the timeline yourself.

clear; close all; clc;

%% --- Options ---------------------------------------------------------------
joint_type  = 'triangle';   % 'triangle' (1 bar/gap) or 'Y' (hub + 2 prongs)
use_control = false;        % true -> engage decentralised joint PID

%% --- Paths -----------------------------------------------------------------
baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir, '00_SourceCode_Solver')));

%% --- Build the model -------------------------------------------------------
opts = struct('R',0.50,'barA',1e-4,'E_panel',70e9,'E_joint',1e8, ...
              'm_node',10.0,'joint_type',joint_type);
model    = Build_Hex_Ring_Model(opts);
assembly = model.assembly;
nNodes   = size(model.node_coords,1);

%% --- Loading & solver setup ------------------------------------------------
F_mag   = 150.0;    % out-of-plane (+Y) impulse force (N)
t_force = 1.0;      % force-on duration (s)
t_total = 8.0;      % total simulation time (s)
dt      = 0.01;
alpha_d = 0.20;  beta_d = 0.0;

totalSteps = round(t_total / dt);
stepForce  = round(t_force / dt);

% Out-of-plane (+Y, component 2) impulse on the outer nodes of ring panel 1
Fext = zeros(totalSteps, nNodes, 3);
nFN  = numel(model.outer_ring1);
for n = model.outer_ring1(:)'
    Fext(1:stepForce, n, 2) = F_mag / nFN;
end

% Supports: clamp ring panel 4 fully (X,Y,Z); Y is FREE everywhere else (3-D)
supp = [model.ring4_nodes(:), ones(numel(model.ring4_nodes),3)];
if model.use_hub
    extra = [model.cr_hub(4); model.rr_hub(3); model.rr_hub(4)];
    supp  = [supp; [extra, ones(numel(extra),3)]];
end

%% --- Run -------------------------------------------------------------------
assembly.Initialize_Assembly();
caa = Solver_CAA_Dynamics;
caa.assembly = assembly;  caa.supp = supp;  caa.dt = dt;  caa.Fext = Fext;
caa.alpha = alpha_d;  caa.beta = beta_d;  caa.rotSprTargetAngle = [];
if use_control
    caa.control = struct('bar_ids',model.joint_ids,'Kp',3.0,'Ki',80.0, ...
                         'Kd',0.05,'target_strain',0,'prestrain_limit',0.05, ...
                         't_on',t_force);
end
fprintf('Running %d steps (%.1f s)...\n', totalSteps, t_total);
Uhis = caa.Solve();
fprintf('Done.\n');

%% --- Joint axial-force history (model-consistent) --------------------------
nJoint = numel(model.joint_ids);
jForce = zeros(totalSteps, nJoint);
for i = 1:totalSteps
    Ex = assembly.bar.Solve_Strain(assembly.node, squeeze(Uhis(i,:,:)));
    jForce(i,:) = (model.E_joint * model.barA * Ex(model.joint_ids))';
end

%% --- Launch the interactive viewer -----------------------------------------
viz = struct();
viz.node_coords = model.node_coords;
viz.hex_nodes   = model.hex_nodes;
viz.bar_conn    = model.bar_conn;
viz.joint_ids   = model.joint_ids;
viz.jointForce  = jForce;
viz.Uhis        = Uhis;
viz.dt          = dt;
viz.skip        = 2;          % subsample for smooth interaction
viz.magnify     = 1.0;        % increase to exaggerate small deformations
viz.title       = sprintf('Hex ring — %s joints, out-of-plane load', joint_type);

Interactive_Deformation_Viewer(viz);

fprintf(['\nInteractive viewer open:\n' ...
         '  • drag to orbit, scroll to zoom, pan tool to translate\n' ...
         '  • use the slider or Play to move through time\n' ...
         '  • change "Magnify" to exaggerate the joint deformation\n']);
