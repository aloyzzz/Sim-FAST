%% constant average acceleration method 
% This file implements the constant average acceleration method for dynamic
% simulation. The method support imput external forces and target rotation
% agnles.

classdef Solver_CAA_Dynamics  < handle
    properties
        % the assembly
        assembly

        % storing the support information
        supp
         
        % the applied load in time history
        Fext
        
        % self folding spring time history
        rotSprTargetAngle
        
        % time increment of each step
        dt        
        
        % Rayleigh damping
        alpha=0.0001
        beta=0.0001

        % Actuated bar rest-length history (optional, OPEN-LOOP).
        % Struct with fields:
        %   bar_ids  — (nAct×1) indices into bar.L0_vec to actuate
        %   L0_his   — (steps×nAct) prescribed rest length at each step
        % Leave empty ([]) to disable actuation.
        actuation = []

        % Closed-loop PID controller on actuated bars (optional).
        % While the open-loop "actuation" plays a prescribed rest-length
        % trajectory, "control" instead computes the actuator command at
        % every time step from feedback so the actuators act to counteract
        % external forces / disturbances and hold the structure near its
        % reference shape.
        %
        % Each actuated bar is treated as a linear contraction/extension
        % actuator. Its eigen-(pre)strain is set every step by a discrete
        % PID law that drives the bar's measured strain toward a target:
        %
        %   e(t)        = target_strain - strain_measured(t)
        %   prestrain   = Kp*e + Ki*∫e dt + Kd*de/dt           (per bar)
        %
        % Sign note: a positive joint strain (the disturbance stretches the
        % joint) gives e < 0, hence a negative prestrain command, which
        % raises the bar tension  Sx = E*(Ex - prestrain)  and pulls the
        % panels back together — i.e. negative feedback that rejects the
        % disturbance. The integral term gives zero steady-state strain
        % error against a sustained external force.
        %
        % Struct fields (scalars are broadcast to all actuated bars):
        %   bar_ids          — (nAct×1) indices of the actuated bars
        %   Kp, Ki, Kd       — PID gains (scalar or nAct×1)
        %   target_strain    — desired bar strain (default 0; scalar/nAct×1)
        %   prestrain_limit  — saturation on |prestrain| modelling finite
        %                      actuator stroke (default Inf; scalar/nAct×1)
        %   t_on             — controller switches on at this time (default 0)
        % Leave empty ([]) to disable closed-loop control.
        control = []

    end
    methods
        % Solve the deformation history. Optional 2nd output ctrlLog
        % returns the controller time histories (empty when control is off).
        [Uhis,ctrlLog]=Solve(obj)

    end
end