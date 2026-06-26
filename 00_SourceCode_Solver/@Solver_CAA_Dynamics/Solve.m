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

        Iacc   = zeros(nAct,1);   % integral accumulator
        ePrev  = zeros(nAct,1);   % previous error (for derivative term)
        ctrlInit = false;

        ctrlLog.bar_ids       = ctrlBars;
        ctrlLog.strain_his    = zeros(step,nAct);  % measured bar strain
        ctrlLog.error_his     = zeros(step,nAct);  % control error
        ctrlLog.prestrain_his = zeros(step,nAct);  % commanded prestrain
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

            if TimeVec(i) >= tOn
                e = eTgt - measEx;                 % control error
                if ~ctrlInit
                    ePrev = e;                     % avoid derivative kick
                    ctrlInit = true;
                end
                deriv = (e - ePrev) / dt;

                % Trial command with the current integral state.
                cmd = Kp.*e + Ki.*Iacc + Kd.*deriv;

                % Conditional integration (anti-windup): only accumulate when
                % the command is not pushing further into saturation.
                notSat = (abs(cmd) < pLim) | (sign(e) ~= sign(cmd));
                Iacc(notSat) = Iacc(notSat) + e(notSat)*dt;

                % Recompute and saturate the command to the stroke limit.
                cmd = Kp.*e + Ki.*Iacc + Kd.*deriv;
                cmd = max(-pLim, min(pLim, cmd));

                ePrev = e;
            else
                e   = eTgt - measEx;
                cmd = zeros(nAct,1);
            end

            assembly.bar.prestrain_vec(ctrlBars) = cmd;

            ctrlLog.strain_his(i,:)    = measEx';
            ctrlLog.error_his(i,:)     = e';
            ctrlLog.prestrain_his(i,:) = cmd';
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