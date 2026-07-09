function Diagnose_SolarPanel_TimeStep()
%% Diagnose_SolarPanel_TimeStep
% Quick passive timestep/damping sweep for the solar-panel model.

clc;

baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir,'00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir,'00_SourceCode_Solver')));

opts.R          = 0.50;
opts.barA       = 1e-4;
opts.E_panel    = 70e9;
opts.E_joint    = 1e8;
opts.t_panel    = 3e-3;
opts.nu_panel   = 0.33;
opts.m_node     = 10.0;
opts.joint_type = 'triangle';

cases = [ ...
    0.010, 0.10, 0.00; ...
    0.005, 0.10, 0.00; ...
    0.002, 0.10, 0.00; ...
    0.001, 0.10, 0.00; ...
    0.005, 1.00, 0.00; ...
    0.005, 0.10, 0.001; ...
    0.002, 1.00, 0.001];

fprintf('dt      alpha   beta      final mean Y (m)   peak mean |Y| (m)\n');
for c = 1:size(cases,1)
    run_case(cases(c,1), cases(c,2), cases(c,3));
end

function run_case(dt, alpha_d, beta_d)
    mdl = Build_SolarPanel_Hex_Model(opts);
    nNodes = size(mdl.node_coords,1);
    panel1_nodes = mdl.hex_nodes(2,:);

    supp = zeros(numel(mdl.ring4_nodes), 4);
    supp(:,1) = mdl.ring4_nodes(:);
    supp(:,2:4) = 1;

    totalSteps = round(4.0 / dt);
    forceSteps = round(1.0 / dt);
    Fext = zeros(totalSteps, nNodes, 3);
    nFN = numel(mdl.outer_ring1);
    for n = mdl.outer_ring1(:)'
        Fext(1:forceSteps, n, 2) = 150.0 / nFN;
    end

    caa = Solver_CAA_Dynamics_DispPID;
    caa.assembly = mdl.assembly;
    caa.supp = supp;
    caa.dt = dt;
    caa.Fext = Fext;
    caa.alpha = alpha_d;
    caa.beta = beta_d;
    Uhis = caa.Solve();

    y = mean(squeeze(Uhis(:, panel1_nodes, 2)), 2);
    fprintf('%0.4f  %0.2f   %0.4f   %+14.6f   %14.6f\n', ...
        dt, alpha_d, beta_d, y(end), max(abs(y)));
end

end
