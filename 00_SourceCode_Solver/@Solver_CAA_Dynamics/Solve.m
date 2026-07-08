function [Uhis,ctrlLog]=Solve(obj)

    % Input setup from loading controller
    assembly=obj.assembly;

    % support information
    supp=obj.supp;

    % time step
    dt=obj.dt;

    % external loading forces
    Fext=obj.Fext;

    % target folding angle
    rotSprTarget=obj.rotSprTargetAngle;

    % We need one more step for Fext 
    % The first step is with zeros
    A=size(Fext);
    step=A(1);
    nodeNum=A(2);

    % adjust the size of Fext 
    Fext0=zeros(1,nodeNum,3);
    Fext=cat(1,Fext0,Fext);

    % vector of every time step
    TimeVec=(1:step)*dt;

    
    % Set up storage         
    Uhis=zeros(step+1,nodeNum,3);
    VHis=Uhis;
    
    V0=zeros(size(assembly.node.current_U_mat));
    
    Uhis(1,:,:)=assembly.node.current_U_mat;
    VHis(1,:,:)=V0;
    
    % The static load that were previouslly applied
    currentAppliedForce=zeros(3*nodeNum,1);    
    for i=1:nodeNum
        currentAppliedForce(3*(i-1)+1:3*i) =...
            assembly.node.current_ext_force_mat(i,:);
    end  
   
    % Find the mass matrix of the system
    MassMat=assembly.node.FindMassMat();

    % --- Closed-loop PID controller set-up ---------------------------------
    % Pre-allocate the controller state and logs. The controller, when
    % enabled, computes the eigen-(pre)strain command for the actuated bars
    % at every step so the actuators counteract external disturbances.
    useCtrl = ~isempty(obj.control);
    ctrlLog = [];
    if useCtrl
        c          = obj.control;
        ctrlBars   = c.bar_ids(:);
        nAct       = numel(ctrlBars);

        % Gains and limits (scalars are broadcast to every actuated bar).
        Kp   = local_col(getfield_default(c,'Kp',0),  nAct);
        Ki   = local_col(getfield_default(c,'Ki',0),  nAct);
        Kd   = local_col(getfield_default(c,'Kd',0),  nAct);
        eTgt = local_col(getfield_default(c,'target_strain',0), nAct);
        pLim = local_col(getfield_default(c,'prestrain_limit',Inf), nAct);
        tOn  = getfield_default(c,'t_on',0);

        % Optional first-order low-pass on the derivative term. The raw
        % derivative (e-ePrev)/dt amplifies step-to-step strain noise, which
        % on a lightly-constrained structure drives the actuators into
        % bang-bang saturation chatter. deriv_tau>0 (seconds) filters it:
        %   dF += (dt/(tau+dt))*(deriv_raw - dF)
        % tau=0 (default) keeps the original unfiltered behaviour.
        derivTau = getfield_default(c,'deriv_tau',0);
        derivF   = zeros(nAct,1);  % filtered-derivative state

        % --- Optional velocity (rate) feedback for active damping ----------
        % An axial actuator removes vibration energy by opposing the rate at
        % which its joint is being stretched. That rate is read here from the
        % solver's node-velocity state (VHis) — the clean modal velocity a
        % real IMU / rate-gyro network would provide — instead of numerically
        % differencing the noisy strain signal (the Kd term). This is a
        % collocated, passive (dissipative) damper, so it damps rather than
        % destabilises. Kv=0 (default) disables it, leaving behaviour intact.
        %   c.Kv          — velocity-damping gain (scalar or nAct×1)
        %   c.vel_sensor  — 'exact' uses VHis directly; 'imu' first collapses
        %                   each panel's node velocities to a rigid-body fit
        %                   (v_centroid + omega×r), modelling what a panel-
        %                   mounted IMU + rate gyro actually senses.
        %   c.imu.panel_nodes — 1×nPanel cell of node-index vectors ('imu')
        Kv        = local_col(getfield_default(c,'Kv',0), nAct);
        velMode   = getfield_default(c,'vel_sensor','exact');
        imuCfg    = getfield_default(c,'imu',[]);
        useVel    = any(Kv ~= 0);
        refCoords = assembly.node.coordinates_mat;
        barNodes  = assembly.bar.node_ij_mat(ctrlBars,:);  % nAct×2 endpoints

        Iacc   = zeros(nAct,1);   % integral accumulator
        ePrev  = zeros(nAct,1);   % previous error (for derivative term)
        ctrlInit = false;

        ctrlLog.bar_ids        = ctrlBars;
        ctrlLog.strain_his     = zeros(step,nAct);  % measured bar strain
        ctrlLog.error_his      = zeros(step,nAct);  % control error
        ctrlLog.prestrain_his  = zeros(step,nAct);  % commanded prestrain
        ctrlLog.strainrate_his = zeros(step,nAct);  % sensed axial strain rate
    end

    % --- Active rotational-joint damping set-up ----------------------------
    useRotDamp = ~isempty(obj.rotDamping) && ...
                 isprop(assembly,'rot_spr_3N') && ~isempty(assembly.rot_spr_3N);
    if useRotDamp
        rs       = assembly.rot_spr_3N;
        rsIJK    = rs.node_ijk_mat;                 % nSpr×3 (i, vertex j, k)
        nSpr     = size(rsIJK,1);
        rsK      = rs.rot_spr_K_vec;
        rdC      = local_col(getfield_default(obj.rotDamping,'C',0), nSpr);
        rdOn     = getfield_default(obj.rotDamping,'t_on',0);
        rdSign   = getfield_default(obj.rotDamping,'sign',1);
        rdTau    = getfield_default(obj.rotDamping,'tau',0.02);  % dθ/dt LP filter
        rsTheta0 = rs.theta_stress_free_vec;        % nominal rest angles
        refCoordsAll = assembly.node.coordinates_mat;
        rdThetaPrev  = rsTheta0;                     % previous hinge angle
        rdDF         = zeros(nSpr,1);                % filtered dθ/dt state
        rdInit       = false;
        if isstruct(ctrlLog)
            rdLog = ctrlLog;
        else
            rdLog = struct();
        end
        rdLog.rot_ijk       = rsIJK;
        rdLog.rot_theta0    = rsTheta0;
        rdLog.rot_dtheta_his = zeros(step,nSpr);    % sensed hinge angular rate
        rdLog.rot_moment_his = zeros(step,nSpr);    % applied damping moment
        ctrlLog = rdLog;
    end

    % Implement the explicit solver
    for i=1:step

        if ~isempty(rotSprTarget)
            assembly.rot_spr_4N.theta_stress_free_vec=rotSprTarget(i,:)';
        end
        if ~isempty(obj.actuation)
            L0_i   = obj.actuation.L0_his(i,:)';
            L0_nat = obj.actuation.L0_nat;
            assembly.bar.prestrain_vec(obj.actuation.bar_ids) = (L0_i - L0_nat) ./ L0_nat;
        end

        % --- Closed-loop PID actuation -------------------------------------
        if useCtrl
            Ui = squeeze(Uhis(i,:,:));
            % Measure current engineering strain of each actuated bar.
            ExAll  = assembly.bar.Solve_Strain(assembly.node, Ui);
            measEx = ExAll(ctrlBars);

            % Sense each actuated joint's axial strain rate from the node
            % velocity state (optionally IMU rigid-body-filtered per panel).
            sRate = zeros(nAct,1);
            if useVel
                Vi = squeeze(VHis(i,:,:));
                if strcmp(velMode,'imu') && ~isempty(imuCfg)
                    Vi = local_imu_rigid(Vi, refCoords + Ui, imuCfg.panel_nodes);
                end
                Pi = refCoords + Ui;
                dP = Pi(barNodes(:,2),:) - Pi(barNodes(:,1),:);   % nAct×3
                Lb = sqrt(sum(dP.^2,2));
                dV = Vi(barNodes(:,2),:) - Vi(barNodes(:,1),:);
                axialV = sum(dV.*dP,2) ./ max(Lb,eps);   % (v_j-v_i)·nhat  [m/s]
                sRate  = axialV ./ max(Lb,eps);          % ≈ d(strain)/dt
            end

            if TimeVec(i) >= tOn
                e = eTgt - measEx;                 % control error
                if ~ctrlInit
                    ePrev = e;                     % avoid derivative kick
                    ctrlInit = true;
                end
                derivRaw = (e - ePrev) / dt;
                if derivTau > 0
                    derivF = derivF + (dt/(derivTau+dt))*(derivRaw - derivF);
                    deriv  = derivF;
                else
                    deriv  = derivRaw;
                end

                % Trial command: PID on strain + clean velocity damping.
                % (-Kv*sRate opposes the joint's stretching velocity.)
                cmd = Kp.*e + Ki.*Iacc + Kd.*deriv - Kv.*sRate;

                % Conditional integration (anti-windup): only accumulate when
                % the command is not pushing further into saturation.
                notSat = (abs(cmd) < pLim) | (sign(e) ~= sign(cmd));
                Iacc(notSat) = Iacc(notSat) + e(notSat)*dt;

                % Recompute and saturate the command to the stroke limit.
                cmd = Kp.*e + Ki.*Iacc + Kd.*deriv - Kv.*sRate;
                cmd = max(-pLim, min(pLim, cmd));

                ePrev = e;
            else
                e   = eTgt - measEx;
                cmd = zeros(nAct,1);
            end

            assembly.bar.prestrain_vec(ctrlBars) = cmd;

            ctrlLog.strain_his(i,:)     = measEx';
            ctrlLog.error_his(i,:)      = e';
            ctrlLog.prestrain_his(i,:)  = cmd';
            ctrlLog.strainrate_his(i,:) = sRate';
        end

        % --- Active rotational-joint damping -------------------------------
        % Inject M = -sign*C*dθ/dt at each hinge by offsetting its stress-free
        % angle. dθ/dt is the hinge angular rate a gyro/encoder would report,
        % obtained by differencing the well-defined joint angle (Solve_Theta)
        % and low-pass filtering it (the closed-form acos-derivative is
        % singular near θ=0/π, so we don't use it).
        if useRotDamp && TimeVec(i) >= rdOn
            Pui = refCoordsAll + squeeze(Uhis(i,:,:));
            thNow = zeros(nSpr,1);
            for si = 1:nSpr
                jn = rsIJK(si,2);
                a  = Pui(rsIJK(si,1),:) - Pui(jn,:);
                b  = Pui(rsIJK(si,3),:) - Pui(jn,:);
                na = norm(a); nb = norm(b);
                if na < eps || nb < eps, thNow(si) = rdThetaPrev(si); continue; end
                thNow(si) = real(acos( dot(a,b)/(na*nb) ));
            end
            if ~rdInit, rdThetaPrev = thNow; rdInit = true; end
            dthRaw = (thNow - rdThetaPrev) / dt;
            if rdTau > 0
                rdDF = rdDF + (dt/(rdTau+dt))*(dthRaw - rdDF);
                dth  = rdDF;
            else
                dth  = dthRaw;
            end
            rdThetaPrev = thNow;
            rs.theta_stress_free_vec = rsTheta0 + rdSign*(rdC ./ max(rsK,eps)).*dth;
            ctrlLog.rot_dtheta_his(i,:) = dth';
            ctrlLog.rot_moment_his(i,:) = (-rdSign*rdC.*dth)';
        end

        [T,K]=assembly.Solve_FK(squeeze(Uhis(i,:,:)));

        [K,T]=Mod_K_For_Supp(K,supp,T);

        [K,Fexti]=Mod_K_For_Supp(K,supp,...
            reshape(squeeze(Fext(i,:,:))',[3*nodeNum,1]));
        [K,Fexti1]=Mod_K_For_Supp(K,supp,...
            reshape(squeeze(Fext(i+1,:,:))',[3*nodeNum,1]));
        
        [K,Vhisi]=Mod_K_For_Supp(K,supp,...
            reshape(squeeze(VHis(i,:,:))',[3*nodeNum,1]));
        [K,Uhisi]=Mod_K_For_Supp(K,supp,...
            reshape(squeeze(Uhis(i,:,:))',[3*nodeNum,1]));

        K=sparse(K);

            
        % Set up the damping matrix
        alpha=obj.alpha;
        beta=obj.beta;
        DampMat=alpha*MassMat+beta*K;
        
        
        % Solve the acceleration
        UDotDot_i=MassMat\(Fexti-DampMat*Vhisi-T);
        
        Kadjust=K+2/dt*DampMat+4/dt/dt*MassMat;
        dP_adjust=(Fexti1-Fexti)+2*DampMat*Vhisi...
            +MassMat*(4/dt*Vhisi+2*UDotDot_i);
        
        Uhisi1=Kadjust\dP_adjust+Uhisi;
        
        Vhisi1=2/dt*(Uhisi1-Uhisi)-Vhisi;
        
        Uhis(i+1,:,:)=reshape(Uhisi1,[3,nodeNum])';
        VHis(i+1,:,:)=reshape(Vhisi1,[3,nodeNum])';
        
        if rem(i,1000)==0
            fprintf('finish solving %d step \n',i);
        end

        % Report progress (~50 updates over the run) to any listener.
        if ~isempty(obj.progressFcn) && ...
                (rem(i, max(1,floor(step/50)))==0 || i==step)
            obj.progressFcn(i/step);
        end

    end

    Uhis=Uhis(1:step,:,:);

end

% --- Local helpers ---------------------------------------------------------
function val = getfield_default(s, name, default)
    % Return s.(name) if it exists and is non-empty, otherwise default.
    if isfield(s, name) && ~isempty(s.(name))
        val = s.(name);
    else
        val = default;
    end
end

function col = local_col(val, n)
    % Expand a scalar to an n×1 column, or pass through an n×1 vector.
    if isscalar(val)
        col = val * ones(n,1);
    else
        col = val(:);
    end
end

function Vout = local_imu_rigid(V, P, panelNodes)
    % Model a panel IMU + rate gyro: collapse each panel's node velocities to
    % the best-fit rigid-body field  v_node = v_centroid + omega × r , which
    % is the motion an IMU/gyro on that (near-rigid) panel actually senses —
    % rejecting intra-panel elastic flex. Least-squares angular velocity:
    %   M*omega = sum(r_k × u_k),   M = sum(|r_k|^2 I - r_k r_k^T)
    % with r_k the node offset from the panel centroid and u_k its velocity
    % residual. Nodes not in any panel keep their raw velocity.
    Vout = V;
    for pnl = 1:numel(panelNodes)
        idx = panelNodes{pnl}(:);
        if numel(idx) < 2, continue; end
        Pc = mean(P(idx,:),1);
        Vc = mean(V(idx,:),1);
        r  = P(idx,:) - Pc;
        u  = V(idx,:) - Vc;
        M  = zeros(3);  b = zeros(3,1);
        for k = 1:numel(idx)
            rk = r(k,:)';  uk = u(k,:)';
            M  = M + (rk'*rk)*eye(3) - rk*rk';
            b  = b + cross(rk, uk);
        end
        if rcond(M) < 1e-12
            omega = [0;0;0];
        else
            omega = M \ b;
        end
        Vout(idx,:) = Vc + cross(repmat(omega',numel(idx),1), r, 2);
    end
end