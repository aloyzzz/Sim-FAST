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

        % Active rotational-joint damping on the 3-node rotational springs
        % (optional). Each joint spring gets a damping moment  M = -C*dθ/dt
        % that opposes the rate of change of its hinge angle, injected by
        % offsetting the spring's stress-free angle every step:
        %     θ_stress_free = θ0 + (C/K)*dθ/dt   ⇒   M = K(θ-θ0) - C*dθ/dt
        % The angular rate dθ/dt is computed analytically from the node
        % velocity state (what a rate gyro across the hinge would sense),
        % not by differencing the angle. Because the moment is collocated
        % with — and directly opposes — the joint rotation, it is passive
        % (dissipative) and targets the ring's panel-swing mode head-on,
        % which the axial bar actuators cannot reach.
        % Struct fields:
        %   C     — rotational damping coeff (N*m*s/rad), scalar or nSpr×1
        %   t_on  — damping switches on at this time (default 0)
        %   sign  — +1 or -1 to match the element's moment convention
        %           (default +1; flip if it adds energy instead of removing)
        % Leave empty ([]) to disable. Requires assembly.rot_spr_3N.
        rotDamping = []

        % Full closed-loop PID on the 3-node rotational joint springs
        % (optional). Where rotDamping supplies rate feedback only, this drives
        % each hinge angle to a setpoint with a commanded moment
        %
        %   e(t) = θ_target - θ(t)
        %   M    = sign * ( Kp*e + Ki*∫e dt + Kd*de/dt )
        %
        % injected the same way rotDamping injects its moment — by offsetting
        % the spring's stress-free angle, θ_stress_free = θ0 - M/K, so that the
        % element returns  M_spring = K(θ-θ0) + M.  Kp adds hinge stiffness, Ki
        % removes the steady hinge deflection a sustained external force leaves
        % behind, Kd damps. The two blocks compose: if both are set their
        % moments are summed before the offset is applied, so a Kd here is the
        % same physical knob as rotDamping.C (identical at equal sign).
        %
        % Struct fields (scalars are broadcast to all nSpr springs):
        %   Kp, Ki, Kd    — PID gains (Kp,Ki in N*m/rad·[s]; Kd in N*m*s/rad)
        %   target_angle  — hinge setpoint (default: each spring's stress-free
        %                   angle θ0, i.e. hold the undeformed geometry)
        %   t_on          — controller switches on at this time (default 0)
        %   sign          — +1/-1 to match the element's moment convention
        %                   (default -1, matching this repo's rot-spring sign)
        %   moment_limit  — saturation on |M| (N*m), modelling finite hinge
        %                   actuator authority (default Inf). The integrator
        %                   stops accumulating while saturated (anti-windup).
        %   tau           — low-pass time constant for the measured hinge rate
        %                   feeding Kd (default 0.02 s; 0 disables filtering)
        % Leave empty ([]) to disable. Requires assembly.rot_spr_3N.
        rotControl = []

        % Optional progress callback, invoked periodically during the time-
        % stepping loop as progressFcn(frac) with frac = i/step in [0,1].
        % Used by GUIs to drive a progress bar. Leave empty ([]) to disable.
        progressFcn = []

    end
    methods
        % Solve the deformation history. Optional 2nd output ctrlLog
        % returns the controller time histories (empty when control is off).
        [Uhis,ctrlLog]=Solve(obj)

    end
end