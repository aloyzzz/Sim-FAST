%% Solver_CAA_Dynamics_DispPID
%
% Extends Solver_CAA_Dynamics with a displacement-based PID controller.
%
% The existing Solver_CAA_Dynamics controls actuated bars using local
% joint-bar strain as feedback. This solver adds a displacement-based
% alternative: sensor nodes are tracked and the position error (reference
% minus measured displacement) drives the actuator prestrains through a
% PID law and a control allocation matrix.
%
% Why displacement feedback?
%   Strain feedback is a LOCAL measurement — it only detects deformation
%   along the actuated bar's axis. A distributed external force that bends
%   panel interiors without heavily loading the joint bars can go largely
%   undetected by the strain controller. Displacement feedback is a GLOBAL
%   shape-error signal: it catches any deformation that shifts sensor nodes
%   away from their reference positions, regardless of load path.
%
% The two controllers can run simultaneously; their prestrain commands are
% summed, enabling cascade or sensor-fusion strategies.

classdef Solver_CAA_Dynamics_DispPID < Solver_CAA_Dynamics

    properties
        % Displacement-based closed-loop PID controller (optional).
        %
        % Measures the displacement of designated sensor nodes and drives
        % their position error to zero by commanding eigenstrain (prestrain)
        % in the actuated bars through a PID law:
        %
        %   e(t)   = U_ref - U_measured(t)      [nSens*nDof vector]
        %   cmd(t) = B_alloc * (Kp*e + Ki*∫e dt + Kd*de/dt)
        %
        % The control allocation matrix B_alloc maps the stacked sensor-
        % displacement error vector to per-actuator prestrain commands.
        % When left empty it is auto-built from geometry: each actuator bar
        % is weighted toward sensor nodes by inverse distance from the bar
        % midpoint, so nearby sensors exert the most influence.
        %
        % Struct fields (scalars broadcast to all sensor DOFs):
        %   sensor_node_ids  — (nSens×1) nodes whose displacement is sensed
        %   sensor_dofs      — DOF indices to sense, e.g. [1 3] for X,Z;
        %                      [2] for out-of-plane Y.  (1-indexed, 1=X 2=Y 3=Z)
        %   actuator_bar_ids — (nAct×1) bar indices to command prestrain on
        %   B_alloc          — (nAct × nSens*nDof) control allocation matrix;
        %                      [] → auto-built from bar-midpoint to sensor-node
        %                      inverse-distance weighting (rows normalised to 1)
        %   Kp, Ki, Kd       — PID gains (scalar or nSens*nDof vector)
        %   U_ref            — reference displacement for sensor DOFs
        %                      (nSens*nDof × 1), default zeros (hold shape)
        %   prestrain_limit  — stroke saturation: |cmd| ≤ limit (default Inf)
        %   actuator_mode    — 'bidirectional' (default), 'contraction', or
        %                      'extension'. Use 'contraction' for cable-like
        %                      joint bars where negative prestrain tensions
        %                      the array and positive prestrain would push
        %                      the joint into compression.
        %   t_on             — controller switches on at this time (default 0)
        %
        % Leave empty ([]) to disable displacement-based control.
        disp_control = []
    end

    methods
        % Override Solve to add displacement PID.
        % Returns displacement history and extended ctrlLog.
        [Uhis, ctrlLog] = Solve(obj)
    end

end
