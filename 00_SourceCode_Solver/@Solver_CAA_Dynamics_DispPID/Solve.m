function [Uhis, ctrlLog] = Solve(obj)
%% Solver_CAA_Dynamics_DispPID.Solve
%
% Constant Average Acceleration dynamic integration with optional:
%   (a) displacement-based PID  (obj.disp_control)
%   (b) strain-based PID        (obj.control, inherited)
%   (c) open-loop actuation     (obj.actuation, inherited)
%   (d) rotational spring targets (obj.rotSprTargetAngle, inherited)
%
% When both (a) and (b) are active their prestrain commands are summed
% before being applied to the assembly, allowing cascade control.

    assembly    = obj.assembly;
    supp        = obj.supp;
    dt          = obj.dt;
    Fext        = obj.Fext;
    rotSprTarget = obj.rotSprTargetAngle;

    A       = size(Fext);
    step    = A(1);
    nodeNum = A(2);

    Fext0 = zeros(1, nodeNum, 3);
    Fext  = cat(1, Fext0, Fext);

    TimeVec = (1:step) * dt;

    Uhis = zeros(step+1, nodeNum, 3);
    VHis = Uhis;

    V0          = zeros(size(assembly.node.current_U_mat));
    Uhis(1,:,:) = assembly.node.current_U_mat;
    VHis(1,:,:) = V0;

    currentAppliedForce = zeros(3*nodeNum, 1);
    for i = 1:nodeNum
        currentAppliedForce(3*(i-1)+1:3*i) = ...
            assembly.node.current_ext_force_mat(i,:);
    end

    MassMat = assembly.node.FindMassMat();

    % --- Strain-based PID setup (inherited from Solver_CAA_Dynamics) -------
    useStrainCtrl = ~isempty(obj.control);
    if useStrainCtrl
        sc       = obj.control;
        sBars    = sc.bar_ids(:);
        nActS    = numel(sBars);
        KpS  = local_col(getf(sc,'Kp',0),  nActS);
        KiS  = local_col(getf(sc,'Ki',0),  nActS);
        KdS  = local_col(getf(sc,'Kd',0),  nActS);
        eTgtS  = local_col(getf(sc,'target_strain',0), nActS);
        pLimS  = local_col(getf(sc,'prestrain_limit',Inf), nActS);
        tOnS   = getf(sc,'t_on',0);
        IaccS  = zeros(nActS,1);
        ePrevS = zeros(nActS,1);
        sCtrlInit = false;
    end

    % --- Displacement-based PID setup --------------------------------------
    useDispCtrl = ~isempty(obj.disp_control);
    if useDispCtrl
        dc       = obj.disp_control;
        sensNodes = dc.sensor_node_ids(:);
        sensDofs  = dc.sensor_dofs(:)';       % row vector of DOF indices
        actBarsD  = dc.actuator_bar_ids(:);
        nActD     = numel(actBarsD);
        nSens     = numel(sensNodes);
        nDof      = numel(sensDofs);
        nSensVec  = nSens * nDof;

        KpD   = local_col(getf(dc,'Kp',0),   nSensVec);
        KiD   = local_col(getf(dc,'Ki',0),   nSensVec);
        KdD   = local_col(getf(dc,'Kd',0),   nSensVec);
        pLimD = local_col(getf(dc,'prestrain_limit',Inf), nActD);
        tOnD  = getf(dc,'t_on',0);
        actuatorModeD = lower(char(getf(dc,'actuator_mode','bidirectional')));

        % Reference displacement vector
        if isfield(dc,'U_ref') && ~isempty(dc.U_ref)
            Uref = dc.U_ref(:);
        else
            Uref = zeros(nSensVec, 1);
        end

        % Build / validate control allocation matrix
        if isfield(dc,'B_alloc') && ~isempty(dc.B_alloc)
            B_alloc = dc.B_alloc;
        else
            B_alloc = local_build_B(assembly, actBarsD, sensNodes, sensDofs);
        end

        IaccD    = zeros(nSensVec, 1);
        ePrevD   = zeros(nSensVec, 1);
        dCtrlInit = false;
    end

    % --- Logging -----------------------------------------------------------
    ctrlLog = struct();
    if useStrainCtrl
        ctrlLog.strain.bar_ids       = sBars;
        ctrlLog.strain.strain_his    = zeros(step, nActS);
        ctrlLog.strain.error_his     = zeros(step, nActS);
        ctrlLog.strain.prestrain_his = zeros(step, nActS);
    end
    if useDispCtrl
        ctrlLog.disp.sensor_node_ids = sensNodes;
        ctrlLog.disp.sensor_dofs     = sensDofs;
        ctrlLog.disp.actuator_bar_ids = actBarsD;
        ctrlLog.disp.disp_his        = zeros(step, nSensVec);
        ctrlLog.disp.error_his       = zeros(step, nSensVec);
        ctrlLog.disp.prestrain_his   = zeros(step, nActD);
    end

    % --- Main time-stepping loop -------------------------------------------
    for i = 1:step

        if ~isempty(rotSprTarget)
            assembly.rot_spr_4N.theta_stress_free_vec = rotSprTarget(i,:)';
        end
        if ~isempty(obj.actuation)
            L0_i   = obj.actuation.L0_his(i,:)';
            L0_nat = obj.actuation.L0_nat;
            assembly.bar.prestrain_vec(obj.actuation.bar_ids) = ...
                (L0_i - L0_nat) ./ L0_nat;
        end

        Ui = squeeze(Uhis(i,:,:));   % (nodeNum × 3)

        % --- (a) Displacement-based PID ------------------------------------
        if useDispCtrl
            % Stack sensor measurements: [U(s1,d1); U(s1,d2); ... U(sN,dM)]
            meas = reshape(Ui(sensNodes, sensDofs), [], 1);
            eD   = Uref - meas;

            if TimeVec(i) >= tOnD
                if ~dCtrlInit
                    ePrevD = eD;
                    dCtrlInit = true;
                end
                deriv = (eD - ePrevD) / dt;

                rawPID = KpD.*eD + KiD.*IaccD + KdD.*deriv;
                cmdD   = B_alloc * rawPID;        % (nActD × 1)
                cmdTrial = local_saturate_cmd(cmdD, pLimD, actuatorModeD);

                % Anti-windup: the integrator lives in sensor space (IaccD)
                % while saturation is defined in actuator space (cmdD, pLimD),
                % so per-element conditional integration isn't well defined
                % here. We freeze ALL sensor integrators whenever ANY actuator
                % is at its stroke limit. This is conservative (a single
                % saturated actuator halts integration for every DOF) but
                % guarantees no windup.
                if all(abs(cmdD - cmdTrial) <= 1e-12)
                    IaccD = IaccD + eD * dt;
                end

                rawPID = KpD.*eD + KiD.*IaccD + KdD.*deriv;
                cmdD   = B_alloc * rawPID;
                cmdD   = local_saturate_cmd(cmdD, pLimD, actuatorModeD);
                ePrevD = eD;
            else
                eD   = Uref - meas;
                cmdD = zeros(nActD, 1);
            end

            assembly.bar.prestrain_vec(actBarsD) = cmdD;

            ctrlLog.disp.disp_his(i,:)     = meas';
            ctrlLog.disp.error_his(i,:)    = eD';
            ctrlLog.disp.prestrain_his(i,:) = cmdD';
        end

        % --- (b) Strain-based PID (adds on top of displacement command) ----
        if useStrainCtrl
            ExAll  = assembly.bar.Solve_Strain(assembly.node, Ui);
            measEx = ExAll(sBars);

            if TimeVec(i) >= tOnS
                eS = eTgtS - measEx;
                if ~sCtrlInit
                    ePrevS = eS;
                    sCtrlInit = true;
                end
                derivS = (eS - ePrevS) / dt;

                cmdS = KpS.*eS + KiS.*IaccS + KdS.*derivS;
                notSatS = (abs(cmdS) < pLimS) | (sign(eS) ~= sign(cmdS));
                IaccS(notSatS) = IaccS(notSatS) + eS(notSatS)*dt;
                cmdS = KpS.*eS + KiS.*IaccS + KdS.*derivS;
                cmdS = max(-pLimS, min(pLimS, cmdS));
                ePrevS = eS;
            else
                eS   = eTgtS - measEx;
                cmdS = zeros(nActS, 1);
            end

            % Assign (not accumulate) the strain command, summed with any
            % displacement command acting on the same bars this step. Using
            % '=' rather than '+=' is essential: cmdS is an ABSOLUTE command,
            % so a running '+=' would turn prestrain_vec into a second,
            % unbounded integrator whenever the displacement loop is inactive.
            dispPart = zeros(nActS, 1);
            if useDispCtrl
                [shared, loc] = ismember(sBars, actBarsD);
                dispPart(shared) = cmdD(loc(shared));
            end
            assembly.bar.prestrain_vec(sBars) = dispPart + cmdS;

            ctrlLog.strain.strain_his(i,:)    = measEx';
            ctrlLog.strain.error_his(i,:)     = eS';
            ctrlLog.strain.prestrain_his(i,:) = cmdS';
        end

        % --- CAA integration step ------------------------------------------
        [T, K] = assembly.Solve_FK(Ui);

        [K, T]      = Mod_K_For_Supp(K, supp, T);
        [K, Fexti]  = Mod_K_For_Supp(K, supp, ...
            reshape(squeeze(Fext(i,:,:))',   [3*nodeNum,1]));
        [K, Fexti1] = Mod_K_For_Supp(K, supp, ...
            reshape(squeeze(Fext(i+1,:,:))', [3*nodeNum,1]));
        [K, Vhisi]  = Mod_K_For_Supp(K, supp, ...
            reshape(squeeze(VHis(i,:,:))',   [3*nodeNum,1]));
        [K, Uhisi]  = Mod_K_For_Supp(K, supp, ...
            reshape(squeeze(Uhis(i,:,:))',   [3*nodeNum,1]));

        K = sparse(K);

        alpha   = obj.alpha;
        beta    = obj.beta;
        DampMat = alpha*MassMat + beta*K;

        UDotDot_i = MassMat \ (Fexti - DampMat*Vhisi - T);

        Kadjust  = K + 2/dt*DampMat + 4/dt/dt*MassMat;
        dP_adjust = (Fexti1 - Fexti) + 2*DampMat*Vhisi + ...
                    MassMat*(4/dt*Vhisi + 2*UDotDot_i);

        Uhisi1 = Kadjust \ dP_adjust + Uhisi;
        Vhisi1 = 2/dt*(Uhisi1 - Uhisi) - Vhisi;

        Uhis(i+1,:,:) = reshape(Uhisi1, [3, nodeNum])';
        VHis(i+1,:,:) = reshape(Vhisi1, [3, nodeNum])';

        if rem(i, 1000) == 0
            fprintf('finish solving %d step\n', i);
        end

        % Report progress (~50 updates over the run) to any listener.
        if ~isempty(obj.progressFcn) && ...
                (rem(i, max(1,floor(step/50)))==0 || i==step)
            obj.progressFcn(i/step);
        end
    end

    Uhis = Uhis(1:step, :, :);

    % If neither controller was used, return empty ctrlLog for compatibility
    if ~useStrainCtrl && ~useDispCtrl
        ctrlLog = [];
    end

end

% =========================================================================
% Local helpers
% =========================================================================

function B = local_build_B(assembly, actBarIds, sensNodes, sensDofs)
% Auto-build control allocation matrix from bar midpoints to sensor nodes.
% B(k,j) = normalized inverse distance from actuator-bar-k midpoint to
% sensor-node-s_j in the plane of DOF d_j.

    coords   = assembly.node.coordinates_mat;   % (nNodes × 3)
    barConn  = assembly.bar.node_ij_mat;         % (nBars  × 2)

    nAct    = numel(actBarIds);
    nSens   = numel(sensNodes);
    nDof    = numel(sensDofs);
    nSensV  = nSens * nDof;

    B = zeros(nAct, nSensV);

    for k = 1:nAct
        bk = actBarIds(k);
        n1 = barConn(bk, 1);
        n2 = barConn(bk, 2);
        mid = (coords(n1,:) + coords(n2,:)) / 2;   % bar midpoint (1×3)

        % Column order MUST match the measurement/error vector, which is
        % built as reshape(Ui(sensNodes, sensDofs), [], 1) — i.e. column-
        % major: DOF is the outer (slower) index, sensor the inner. Iterate
        % accordingly so B_alloc columns align with the error entries.
        col = 0;
        for d = 1:nDof
            for s = 1:nSens
                sCoord = coords(sensNodes(s), :);
                dist   = norm(mid - sCoord);
                col = col + 1;
                B(k, col) = 1 / max(dist, 1e-12);
            end
        end
        rowSum = sum(B(k,:));
        if rowSum > 0
            B(k,:) = B(k,:) / rowSum;
        end
    end
end

function val = getf(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        val = s.(name);
    else
        val = default;
    end
end

function col = local_col(val, n)
    if isscalar(val)
        col = val * ones(n,1);
    else
        col = val(:);
    end
end

function cmd = local_saturate_cmd(cmd, pLim, actuatorMode)
    switch actuatorMode
        case 'bidirectional'
            cmd = max(-pLim, min(pLim, cmd));
        case 'contraction'
            % Negative prestrain shortens the bar and creates tension.
            % This is the physically appropriate mode for cable-like joints:
            % they may tense the array, but should not actively push in
            % compression when the displacement error changes sign.
            cmd = max(-pLim, min(zeros(size(cmd)), cmd));
        case 'extension'
            cmd = max(zeros(size(cmd)), min(pLim, cmd));
        otherwise
            error('Solver_CAA_Dynamics_DispPID:BadActuatorMode', ...
                'disp_control.actuator_mode must be bidirectional, contraction, or extension.');
    end
end
