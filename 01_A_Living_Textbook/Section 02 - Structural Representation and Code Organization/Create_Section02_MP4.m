%% Generate truss_deformation.mp4 for the Section 02 example
%  Run this script from the Section 02 directory (or with it on the path).
%  Requires the source folders 00_SourceCode_Elements and
%  00_SourceCode_Solver to be on the MATLAB path.

clear; close all; clc;

% -------------------------------------------------------------------------
% Build the truss assembly
% -------------------------------------------------------------------------
assembly = Assembly_Truss;

node   = Elements_Nodes;
bar    = Vec_Elements_Bars;
actBar = CD_Elements_Bars;

assembly.node   = node;
assembly.bar    = bar;
assembly.actBar = actBar;

L = 1;  % span length (m)

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

barA = 0.0001;      % cross-section area  (m^2)
barE = 2e9;         % Young's modulus     (Pa)
actE = 2e6;         % active bar modulus  (Pa)

bar.A_vec    = barA * ones(7, 1);
bar.E_vec    = barE * ones(7, 1);
actBar.A_vec = barA * ones(2, 1);
actBar.E_vec = actE * ones(2, 1);

assembly.Initialize_Assembly();

% -------------------------------------------------------------------------
% Solve for the deformation history
% -------------------------------------------------------------------------
deltaL = 0.3;  % actuation stroke (m)

nr           = Solver_NR_TrussAction;
nr.assembly  = assembly;
nr.supp      = [1 1 1 1;
                2 1 1 1;
                3 0 1 0;
                4 0 1 0;
                5 0 1 0;
                6 0 1 0];

nr.targetL0     = actBar.L0_vec;
nr.targetL0(1)  = nr.targetL0(1) - deltaL;   % contract bar 1
nr.targetL0(2)  = nr.targetL0(2) + deltaL;   % extend  bar 2

nr.increStep = 20;
nr.iterMax   = 30;

Uhis = nr.Solve();

% -------------------------------------------------------------------------
% Export MP4
% -------------------------------------------------------------------------
plots               = Plot_Truss();
plots.assembly      = assembly;
plots.displayRange  = [-2; 3; -1; 1; -0.5; 3];
plots.viewAngle1    = 20;
plots.viewAngle2    = 15;
plots.width         = 800;
plots.height        = 600;
plots.holdTime      = 0.05;   % controls frame rate (1/holdTime fps)

mp4File = fullfile(fileparts(mfilename('fullpath')), 'truss_deformation.mp4');
plots.Plot_Deformed_His_MP4(Uhis, mp4File);

fprintf('Done. Video saved to:\n  %s\n', mp4File);
