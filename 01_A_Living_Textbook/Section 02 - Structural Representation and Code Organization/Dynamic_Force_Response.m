%% Dynamic Force Response - Section 02 Truss
%
% Applies a lateral force to the apex node for 1 second, then removes it
% and observes 3 seconds of free vibration. Saves the result as an MP4.
%
% Run from the Section 02 directory (or with it on the path). Requires
% 00_SourceCode_Elements and 00_SourceCode_Solver to be on the MATLAB path.

clear; close all; clc;

% =========================================================================
% Parameters  (adjust to taste)
% =========================================================================
L       = 1;        % span length (m)
barA    = 1e-4;     % cross-section area (m^2)
barE    = 1e6;      % Young's modulus (Pa)  -- soft material for clear demo
actE    = 1e6;      % active-bar modulus (Pa)
m_node  = 0.1;      % lumped mass per node (kg)

F_applied = 5;      % force magnitude applied to apex node (N)
F_dir     = 1;      % direction index: 1=X, 2=Y, 3=Z
F_node    = 6;      % node to load (apex)

t_force = 1.0;      % duration of applied force (s)
t_total = 4.0;      % total simulation time (s)
dt      = 5e-3;     % time step (s)

alpha_damp = 1.0;   % Rayleigh mass-proportional damping coefficient
beta_damp  = 1e-6;  % Rayleigh stiffness-proportional damping coefficient

skipFrames = 4;     % subsample ratio for MP4 (1 = every step)
frameRate  = 20;    % MP4 playback frame rate (fps)

% =========================================================================
% Build the truss assembly (same geometry as Section 02 Example)
% =========================================================================
assembly        = Assembly_Truss;
node            = Elements_Nodes;
bar             = Vec_Elements_Bars;
actBar          = CD_Elements_Bars;
assembly.node   = node;
assembly.bar    = bar;
assembly.actBar = actBar;

node.coordinates_mat = [0      0  0;
                        L      0  0;
                        0      0  L;
                        L      0  L;
                        0.5*L  0  0.5*L;
                        0.5*L  0  2*L  ];

bar.node_ij_mat = [2 5;
                   1 5;
                   3 4;
                   3 5;
                   4 5;
                   3 6;
                   4 6];

actBar.node_ij_mat = [1 3;
                      2 4];

bar.A_vec    = barA * ones(7, 1);
bar.E_vec    = barE * ones(7, 1);
actBar.A_vec = barA * ones(2, 1);
actBar.E_vec = actE * ones(2, 1);

% Assign lumped masses to every node
node.mass_vec = m_node * ones(6, 1);

assembly.Initialize_Assembly();

% =========================================================================
% Build Fext: force on for t_force seconds, then zero
% =========================================================================
nodeNum    = size(node.coordinates_mat, 1);
totalSteps = round(t_total / dt);
stepForce  = round(t_force / dt);

Fext = zeros(totalSteps, nodeNum, 3);
Fext(1:stepForce, F_node, F_dir) = F_applied;

% =========================================================================
% Run the dynamic solver
% =========================================================================
supp = [1 1 1 1;
        2 1 1 1;
        3 0 1 0;
        4 0 1 0;
        5 0 1 0;
        6 0 1 0];

caa              = Solver_CAA_Dynamics();
caa.assembly     = assembly;
caa.supp         = supp;
caa.dt           = dt;
caa.Fext         = Fext;
caa.alpha        = alpha_damp;
caa.beta         = beta_damp;
caa.rotSprTargetAngle = [];   % no rotational springs in this assembly

fprintf('Running dynamic simulation (%d steps)...\n', totalSteps);
Uhis = caa.Solve();
fprintf('Done.\n');

% =========================================================================
% Export MP4
% =========================================================================
Uhis_sub = Uhis(1:skipFrames:end, :, :);   % subsample for smooth video

plots               = Plot_Truss();
plots.assembly      = assembly;
plots.displayRange  = [-0.5; 1.5; -0.5; 0.5; -0.2; 2.5];
plots.viewAngle1    = 20;
plots.viewAngle2    = 15;
plots.width         = 800;
plots.height        = 600;
plots.holdTime      = 1 / frameRate;

mp4File = fullfile(fileparts(mfilename('fullpath')), 'dynamic_response.mp4');
plots.Plot_Deformed_His_MP4(Uhis_sub, mp4File);

fprintf('Video saved to:\n  %s\n', mp4File);

% =========================================================================
% Plot node 6 displacement vs time (quick check)
% =========================================================================
time = (0:totalSteps-1) * dt;
u6x  = squeeze(Uhis(:, F_node, F_dir));

figure;
plot(time, u6x * 1000, 'b', 'LineWidth', 1.5);
hold on;
xline(t_force, 'r--', 'Force off', 'LabelVerticalAlignment', 'bottom');
xlabel('Time (s)');
ylabel(sprintf('Node %d displacement in dir %d (mm)', F_node, F_dir));
title('Dynamic Response: forced (0–1 s) then free vibration (1–4 s)');
grid on;
