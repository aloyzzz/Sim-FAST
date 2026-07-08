function Diagnose_SolarPanel_DispPID()
%% Diagnose_SolarPanel_DispPID
% Headless diagnostics for the solar-panel displacement PID example.

clc;

baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(baseDir,'00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir,'00_SourceCode_Solver')));

R         = 0.50;
barA      = 1e-4;
E_panel   = 70e9;
E_joint   = 1e8;
t_panel   = 3e-3;
nu_panel  = 0.33;
m_node    = 10.0;
F_mag     = 150.0;
t_force   = 1.0;
t_total   = 6.0;
dt        = 0.002;
alpha_d   = 1.00;
beta_d    = 0.001;

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

ring4_nodes = mdl.ring4_nodes;
supp = zeros(numel(ring4_nodes), 4);
supp(:,1) = ring4_nodes(:);
supp(:,2) = 1; supp(:,3) = 1; supp(:,4) = 1;

outer_hex1 = mdl.outer_ring1;
nFN        = numel(outer_hex1);
totalSteps = round(t_total / dt);
forceSteps = round(t_force / dt);

Fext = zeros(totalSteps, nNodes, 3);
for n = outer_hex1(:)'
    Fext(1:forceSteps, n, 2) = F_mag / nFN;
end

panel1_nodes = hex_nodes(2,:);
act_ids = [];
for b = 1:nBars
    if mdl.is_joint(b)
        n1 = bar_conn(b,1); n2 = bar_conn(b,2);
        if any(panel1_nodes == n1) || any(panel1_nodes == n2)
            act_ids(end+1) = b; %#ok<SAGROW>
        end
    end
end
act_ids = act_ids(:);

fprintf('\n--- Initial stiffness diagnostic ---\n');
[T0, K0] = mdl.assembly.Solve_FK(zeros(nNodes,3));
[Kmod, Tmod] = Mod_K_For_Supp(K0, supp, T0); %#ok<ASGLU>
suppDofs = false(3*nNodes,1);
for i = 1:size(supp,1)
    n = supp(i,1);
    for d = 1:3
        if supp(i,d+1)
            suppDofs(3*(n-1)+d) = true;
        end
    end
end
freeDofs = find(~suppDofs);
Kfree = full(K0(freeDofs, freeDofs));
lambda = sort(real(eig((Kfree + Kfree')/2)));
fprintf('free DOFs: %d\n', numel(freeDofs));
fprintf('smallest stiffness eigenvalues:\n');
fprintf('  %.6e\n', lambda(1:min(12,numel(lambda))));
fprintf('initial internal force norm: %.6e\n', norm(T0));

yDofsAll = (2:3:3*nNodes)';
yFree = yDofsAll(~suppDofs(yDofsAll));
Ky = full(K0(yFree, yFree));
lambdaY = sort(real(eig((Ky + Ky')/2)));
Fy = zeros(numel(yFree),1);
for n = outer_hex1(:)'
    dof = 3*(n-1) + 2;
    loc = find(yFree == dof, 1);
    if ~isempty(loc)
        Fy(loc) = F_mag / nFN;
    end
end
uy_static = Ky \ Fy;
panel1YLoc = [];
for n = panel1_nodes(:)'
    panel1YLoc(end+1) = find(yFree == 3*(n-1)+2, 1); %#ok<AGROW>
end
fprintf('smallest Y stiffness eigenvalues:\n');
fprintf('  %.6e\n', lambdaY(1:min(8,numel(lambdaY))));
fprintf('linear static panel-1 mean Y under peak force: %.6e m\n', ...
    mean(uy_static(panel1YLoc)));

fprintf('\n--- Dynamic cases ---\n');
run_case('passive', []);

ctrl_d.sensor_node_ids  = panel1_nodes(:);
ctrl_d.sensor_dofs      = 2;
ctrl_d.actuator_bar_ids = act_ids;
ctrl_d.B_alloc          = [];
ctrl_d.Kp               = 1.5;
ctrl_d.Ki               = 8.0;
ctrl_d.Kd               = 0.20;
ctrl_d.U_ref            = [];
ctrl_d.prestrain_limit  = 0.05;
ctrl_d.actuator_mode    = 'bidirectional';
ctrl_d.t_on             = 1.0;
run_case('disp bidirectional', ctrl_d);

ctrl_d.actuator_mode    = 'contraction';
run_case('disp contraction clamp', ctrl_d);

ctrl_d.t_on             = 0.0;
run_case('disp contraction clamp, on at t=0', ctrl_d);

function run_case(label, ctrl_d)
    mdl.assembly.Initialize_Assembly();
    caa = Solver_CAA_Dynamics_DispPID;
    caa.assembly = mdl.assembly;
    caa.supp = supp;
    caa.dt = dt;
    caa.Fext = Fext;
    caa.alpha = alpha_d;
    caa.beta = beta_d;
    if ~isempty(ctrl_d)
        caa.disp_control = ctrl_d;
    end
    [Uhis, log] = caa.Solve();
    y = mean(squeeze(Uhis(:, panel1_nodes, 2)), 2);
    fprintf('%-34s final %+10.4f m, peak %+10.4f m', ...
        label, y(end), max(abs(y)));
    if ~isempty(ctrl_d) && ~isempty(log)
        cmd = log.disp.prestrain_his;
        fprintf(', cmd [%+.4f, %+.4f]', min(cmd(:)), max(cmd(:)));
    end
    fprintf('\n');
end

end
