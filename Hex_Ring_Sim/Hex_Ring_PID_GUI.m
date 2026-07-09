function Hex_Ring_PID_GUI()
%HEX_RING_PID_GUI  Interactive GUI for the 1-center + 6-ring hex model.
%
%   Drives the rotational-spring hex-center-ring model from
%   Hex_Center_Ring_CONTROL_RotSpr.m in two modes:
%     • Open Loop   — plays a prescribed rest-length trajectory on all 12
%                     joint actuators (reproduces the script).
%     • PID Control — runs passive vs. closed-loop strain-PID, where the
%                     controller drives the LOADED-SIDE joint bars (those
%                     touching ring hex 1) to reject the +X force impulse.
%
%   Select a mode from the dropdown, tweak the parameter fields, then press
%   Run.  Run from the Hex_Ring_Sim directory or any directory that can
%   find the 00_SourceCode_* trees.

%% --- Paths -----------------------------------------------------------------
thisDir = fileparts(mfilename('fullpath'));
baseDir = fileparts(thisDir);
addpath(thisDir);   % for Interactive_Deformation_Viewer + @Assembly_*_RotSpr
addpath(genpath(fullfile(baseDir,'00_SourceCode_Elements')));
addpath(genpath(fullfile(baseDir,'00_SourceCode_Solver')));

%% ---- Top-level figure -----------------------------------------------------
DARK_BG   = [0.11 0.13 0.17];
PANEL_BG  = [0.16 0.19 0.26];
FIELD_BG  = [0.20 0.23 0.32];
WHITE     = [1.00 1.00 1.00];
GOLD      = [0.95 0.78 0.20];
TEAL      = [0.18 0.75 0.70];
GREEN_BTN = [0.14 0.56 0.34];
ACCENT    = [0.35 0.60 0.95]; %#ok<NASGU>

fig = uifigure('Name','Hex Center Ring PID Simulation Control', ...
    'Position',[60 40 1480 880], 'Color',DARK_BG, 'Resize','on', ...
    'AutoResizeChildren','off');

%% ---- Header ---------------------------------------------------------------
header = uipanel(fig,'Position',[0 848 1480 32],'BackgroundColor',PANEL_BG, ...
    'BorderType','none','AutoResizeChildren','off');
headerLbl = uilabel(header,'Text','Hex Center Ring PID Simulation Control', ...
    'Position',[0 0 1480 32], 'FontSize',17,'FontWeight','bold', ...
    'FontColor',GOLD,'HorizontalAlignment','center', ...
    'BackgroundColor',PANEL_BG);

%% ---- Left control panel ---------------------------------------------------
LP_W = 385;
lp = uipanel(fig,'Position',[8 8 LP_W 836],'BackgroundColor',PANEL_BG, ...
    'BorderType','none','Title','','AutoResizeChildren','off');

% Simulation type selector
simLbl = mkLabel(lp,'Simulation',10,804,LP_W-20,20,13,'bold',WHITE);
simDrop = uidropdown(lp, ...
    'Items',{'Hex Center Ring — Open Loop','Hex Center Ring — PID Control'}, ...
    'Value','Hex Center Ring — PID Control', ...
    'Position',[10 776 LP_W-20 26], ...
    'BackgroundColor',FIELD_BG,'FontColor',WHITE,'FontSize',11);

% ---- Tabbed parameter groups -----------------------------------------------
tg = uitabgroup(lp,'Position',[5 258 LP_W-10 514]);
tStruct = uitab(tg,'Title','  Structure  ');
tSim    = uitab(tg,'Title','  Simulation  ');
tPID    = uitab(tg,'Title','  PID Gains  ');

colorTab(tStruct, PANEL_BG);
colorTab(tSim,    PANEL_BG);
colorTab(tPID,    PANEL_BG);

% Layout constants shared by all tabs
C1 = 10; C2 = 200; FW = 162; FH = 28;
ROW = 54;   % label(20) + gap(6) + field(28) = 54 px per row

% ------------------------------------------------------------------
%  STRUCTURE tab
% ------------------------------------------------------------------
y = 430;
ef.R    = mkField(tStruct,'Circumradius R (m)',  0.50, C1,y,FW,FH);
ef.barA = mkField(tStruct,'Bar area (m²)',        1e-4, C2,y,FW,FH); y=y-ROW;

ef.Ep   = mkField(tStruct,'E_panel (Pa)',        70e9, C1,y,FW,FH);
ef.Ej   = mkField(tStruct,'E_joint (Pa)',         1e8, C2,y,FW,FH); y=y-ROW;

ef.mn   = mkField(tStruct,'Node mass (kg)',       10.0, C1,y,FW,FH);
ef.Krot = mkField(tStruct,'K_rot (N·m/rad)',      50.0, C2,y,FW,FH); y=y-ROW;

uilabel(tStruct,'Text','K_rot = rotational-spring joint stiffness (24 springs)', ...
    'Position',[C1 y+18 340 18],'FontColor',[0.55 0.60 0.70], ...
    'FontSize',9,'FontAngle','italic');

% ------------------------------------------------------------------
%  SIMULATION tab
% ------------------------------------------------------------------
y = 430;
ef.Fm   = mkField(tSim,'Force magnitude (N)',  80.0, C1,y,FW,FH);
ef.tf   = mkField(tSim,'Force duration (s)',    2.0, C2,y,FW,FH); y=y-ROW;

ef.ttot = mkField(tSim,'Total time (s)',       12.0, C1,y,FW,FH);
ef.dt   = mkField(tSim,'Time step dt (s)',     0.01, C2,y,FW,FH); y=y-ROW;

ef.alp  = mkField(tSim,'Rayleigh α  (mass)',   0.10, C1,y,FW,FH);
ef.bet  = mkField(tSim,'Rayleigh β  (stiff)',  0.00, C2,y,FW,FH); y=y-ROW;

% Section header sits in its own band; mkField draws its label at y+FH+2,
% so the next row must start well below the header to avoid overlapping it.
mkLabel(tSim,'— Open-loop actuation † —',C1,y+16,340,20,11,'bold',GOLD);
y = y-32;
ef.eact = mkField(tSim,'Actuator strain',     -0.15, C1,y,FW,FH);
ef.tramp= mkField(tSim,'Ramp time (s)',         1.5, C2,y,FW,FH); y=y-ROW;
ef.thold= mkField(tSim,'Hold time (s)',         1.0, C1,y,FW,FH);
uilabel(tSim,'Text','† Open-Loop mode only', ...
    'Position',[C2 y+4 FW 18],'FontColor',[0.55 0.60 0.70], ...
    'FontSize',9,'FontAngle','italic');

% ------------------------------------------------------------------
%  PID GAINS tab   (Strain PID — drives loaded-side joint bars)
% ------------------------------------------------------------------
% A field's own label occupies y+FH+2 .. y+FH+18, so a section header needs
% ~50 px of clearance above the first field it introduces.
HDR = 50;
y = 455;
mkLabel(tPID,'— Disturbance rejection (strain PID) —',C1,y,340,20,10.5,'bold',TEAL); y=y-HDR;
ef.Kps  = mkField(tPID,'Kp  (proportional)',  0.6,  C1,y,FW,FH);
ef.Kis  = mkField(tPID,'Ki  (integral)',      1.0,  C2,y,FW,FH); y=y-ROW;
ef.Kds  = mkField(tPID,'Kd  (derivative)',    0.0,  C1,y,FW,FH);
ef.pls  = mkField(tPID,'Prestrain limit',     0.05, C2,y,FW,FH); y=y-ROW;
ef.tons = mkField(tPID,'Controller on (s)',    1.0, C1,y,FW,FH);
ef.Kv   = mkField(tPID,'Kv  (axial vel damp)', 0.0, C2,y,FW,FH); y=y-ROW-8;

mkLabel(tPID,'— Rotational-joint PID (hinge angle) —',C1,y,340,20,10.5,'bold',GOLD); y=y-HDR;
ef.Kpr  = mkField(tPID,'Kp_rot (N·m/rad)',      0.0, C1,y,FW,FH);
ef.Kir  = mkField(tPID,'Ki_rot (N·m/rad/s)',    0.0, C2,y,FW,FH); y=y-ROW;
ef.Kdr  = mkField(tPID,'Kd_rot (N·m·s/rad)',     50, C1,y,FW,FH);
ef.Mlim = mkField(tPID,'Moment limit (0=∞)',    0.0, C2,y,FW,FH); y=y-ROW;

uilabel(tPID,'Text',['The hinge PID drives each joint angle back to its ' ...
    'undeformed value: M = Kp·e + Ki·∫e + Kd·ė. Kd_rot alone is pure rate ' ...
    'damping (the old C_rot) — it settles the ring mode the axial actuators ' ...
    'can''t reach; Kd_rot≈50 settles cleanly, ≳120 diverges. Kp_rot stiffens ' ...
    'the hinges, Ki_rot removes their steady deflection. Keep the axial ' ...
    'Kd = Kv = 0 when Kd_rot > 0 (the dampers fight otherwise).'], ...
    'Position',[C1 y-58 340 96],'FontColor',[0.55 0.60 0.70], ...
    'FontSize',9,'FontAngle','italic','WordWrap','on');

% ---- Buttons & status (below the tab group) --------------------------------
resetBtn = uibutton(lp,'push','Text','↺  Reset Defaults', ...
    'Position',[10 222 LP_W-20 30], ...
    'BackgroundColor',[0.22 0.30 0.42],'FontColor',WHITE, ...
    'FontSize',11,'FontWeight','bold');

runBtn = uibutton(lp,'push','Text','▶  Run Simulation', ...
    'Position',[10 180 LP_W-20 36], ...
    'BackgroundColor',GREEN_BTN,'FontColor',WHITE, ...
    'FontSize',13,'FontWeight','bold');

statusLbl = uilabel(lp,'Text','Ready — configure parameters and press Run.', ...
    'Position',[10 160 LP_W-20 18], ...
    'FontColor',TEAL,'FontSize',9,'HorizontalAlignment','center');

% ---- Progress bar (track + fill; fill width scales 0..100%) ----------------
PROG_W    = LP_W-20;
progTrack = uipanel(lp,'Position',[10 142 PROG_W 14], ...
    'BackgroundColor',[0.07 0.09 0.12],'BorderType','none');
progFill  = uipanel(progTrack,'Units','pixels','Position',[0 0 1 14], ...
    'BackgroundColor',GREEN_BTN,'BorderType','none');

% ---- Console log (fixed height at bottom)
% scroll(uitextarea,'bottom') scrolls the content clean out of view, so the
% console is kept as a tail: only the newest LOG_MAX lines (as many as fit
% without scrolling) are ever shown. See logAppend.
LOG_H   = 118;
LOG_MAX = max(3, floor((LOG_H - 8) / 12));
mkLabel(lp,'Console Output',10,126,200,16,10,'normal',[0.65 0.68 0.75]);
logBox = uitextarea(lp,'Position',[10 8 LP_W-20 LOG_H], ...
    'Editable','off','BackgroundColor',[0.07 0.09 0.12], ...
    'FontColor',[0.45 0.82 0.50],'FontSize',8.5,'FontName','Courier New');

%% ---- Right plot panel -----------------------------------------------------
RP_X = LP_W + 16;
RP_W = 1480-RP_X-8;
rp = uipanel(fig,'Position',[RP_X 8 RP_W 836], ...
    'BackgroundColor',DARK_BG,'BorderType','none','AutoResizeChildren','off');

% ---- Top strip: pick a case, open the 3-D viewer, or export an MP4 --------
caseLbl = uilabel(rp,'Text','Case:','Position',[6 802 44 24], ...
    'FontColor',WHITE,'FontSize',11,'FontWeight','bold');
caseDrop = uidropdown(rp,'Items',{'(run a simulation first)'}, ...
    'Position',[52 802 224 26],'BackgroundColor',FIELD_BG, ...
    'FontColor',WHITE,'FontSize',11,'Enable','off');
viewBtn = uibutton(rp,'push','Text','🧊  Open 3D Viewer', ...
    'Position',[284 800 200 30],'BackgroundColor',[0.20 0.42 0.62], ...
    'FontColor',WHITE,'FontSize',12,'FontWeight','bold','Enable','off');
exportBtn = uibutton(rp,'push','Text','🎬  Export MP4', ...
    'Position',[492 800 165 30],'BackgroundColor',[0.55 0.30 0.55], ...
    'FontColor',WHITE,'FontSize',12,'FontWeight','bold','Enable','off');
hintLbl = uilabel(rp,'Text','viewer: drag=orbit • scroll=zoom   |   MP4: renders the selected case', ...
    'Position',[665 802 RP_W-673 24],'FontColor',[0.60 0.64 0.72], ...
    'FontSize',9,'FontAngle','italic');

ptab = uitabgroup(rp,'Position',[4 4 RP_W-8 790],'AutoResizeChildren','off');

dispTab = uitab(ptab,'Title','  Displacement History  ','AutoResizeChildren','off');
actTab  = uitab(ptab,'Title','  Actuator Commands  ','AutoResizeChildren','off');
rotTab  = uitab(ptab,'Title','  Joint Moments  ','AutoResizeChildren','off');
colorTab(dispTab, DARK_BG);
colorTab(actTab,  DARK_BG);
colorTab(rotTab,  DARK_BG);

% Height must stay inside the tab's client area (tab group height minus the
% ~30 px title strip), or the axes title is clipped off the top.
AXH = 790 - 30 - 26;
ax_disp = uiaxes(dispTab,'Position',[20 18 RP_W-52 AXH], ...
    'Color',[0.11 0.13 0.19],'XColor',WHITE,'YColor',WHITE, ...
    'GridColor',[0.30 0.33 0.42],'GridAlpha',0.5,'FontSize',10);
title(ax_disp,'Press Run to see results','Color',WHITE,'FontSize',14);
hold(ax_disp,'on'); grid(ax_disp,'on'); box(ax_disp,'on');

ax_act  = uiaxes(actTab,'Position',[20 18 RP_W-52 AXH], ...
    'Color',[0.11 0.13 0.19],'XColor',WHITE,'YColor',WHITE, ...
    'GridColor',[0.30 0.33 0.42],'GridAlpha',0.5,'FontSize',10);
title(ax_act,'Press Run to see results','Color',WHITE,'FontSize',14);
hold(ax_act,'on'); grid(ax_act,'on'); box(ax_act,'on');

ax_rot  = uiaxes(rotTab,'Position',[20 18 RP_W-52 AXH], ...
    'Color',[0.11 0.13 0.19],'XColor',WHITE,'YColor',WHITE, ...
    'GridColor',[0.30 0.33 0.42],'GridAlpha',0.5,'FontSize',10);
title(ax_rot,'Press Run to see results','Color',WHITE,'FontSize',14);
hold(ax_rot,'on'); grid(ax_rot,'on'); box(ax_rot,'on');

% Everything above is placed in pixels, so reflow it whenever the window is
% resized — otherwise the panels stay bottom-left anchored and the plots
% either get clipped or leave a dead band on the right.
fig.SizeChangedFcn = @(~,~) layoutFig();
layoutFig();

%% ---- Callbacks ------------------------------------------------------------
lastResult = [];   % results of the most recent successful run (for 3D viewer)

runBtn.ButtonPushedFcn   = @(~,~) onRun();
resetBtn.ButtonPushedFcn = @(~,~) onReset();
simDrop.ValueChangedFcn  = @(~,~) onSimTypeChanged();
viewBtn.ButtonPushedFcn  = @(~,~) onView();
exportBtn.ButtonPushedFcn = @(~,~) onExport();
onSimTypeChanged();   % set initial field enable states

    %% ---- logAppend / logReset --------------------------------------------
    % Console tail. Only the newest LOG_MAX lines are held in the text area,
    % so the latest message is always on screen without asking uitextarea to
    % scroll (scroll(...,'bottom') pushes the content out of view entirely).
    function logAppend(lines)
        if ~iscell(lines), lines = {lines}; end
        v = [logBox.Value(:); lines(:)];
        v = v(~cellfun(@isempty, v));
        if numel(v) > LOG_MAX, v = v(end-LOG_MAX+1:end); end
        if isempty(v), v = {''}; end   % uitextarea rejects an empty cell
        logBox.Value = v;
    end

    function logReset(lines)
        logBox.Value = {''};
        logAppend(lines);
    end

    %% ---- layoutFig -------------------------------------------------------
    % Reflow the pixel-positioned panels for the current figure size. The left
    % column keeps its fixed width (LP_W) so the progress bar and field grid
    % stay valid; only heights and the right column's width follow the window.
    function layoutFig()
        fp = fig.Position;
        W  = max(fp(3), 900);
        H  = max(fp(4), 520);

        header.Position    = [0 H-32 W 32];
        headerLbl.Position = [0 0 W 32];

        lpH = H - 44;
        lp.Position = [8 8 LP_W lpH];
        simLbl.Position  = [10 lpH-32 LP_W-20 20];
        simDrop.Position = [10 lpH-60 LP_W-20 26];
        tg.Position      = [5 258 LP_W-10 max(120, lpH-64-258)];
        % Buttons, status, progress bar and console stay anchored to the bottom.

        rpW = max(320, W - RP_X - 8);
        rpH = lpH;
        rp.Position = [RP_X 8 rpW rpH];
        caseLbl.Position   = [6   rpH-34 44 24];
        caseDrop.Position  = [52  rpH-34 224 26];
        viewBtn.Position   = [284 rpH-36 200 30];
        exportBtn.Position = [492 rpH-36 165 30];
        hintLbl.Position   = [665 rpH-34 max(10, rpW-673) 24];

        ptabH = max(140, rpH - 46);
        ptab.Position = [4 4 rpW-8 ptabH];
        axW = max(120, rpW-52);
        axH = max(80, ptabH - 30 - 26);
        ax_disp.Position = [20 18 axW axH];
        ax_act.Position  = [20 18 axW axH];
        ax_rot.Position  = [20 18 axW axH];
    end

    %% ---- onSimTypeChanged ------------------------------------------------
    function onSimTypeChanged()
        isPID = contains(simDrop.Value,'PID');
        % Open-loop fields active only in Open Loop mode.
        setEnable({ef.eact, ef.tramp, ef.thold}, ~isPID);
        % PID-gain fields active only in PID mode.
        setEnable({ef.Kps, ef.Kis, ef.Kds, ef.pls, ef.tons, ef.Kv, ...
                   ef.Kpr, ef.Kir, ef.Kdr, ef.Mlim}, isPID);
        % Open-Loop matches the script's pure-actuation demo (no external
        % force); PID needs a disturbance to reject, so default to 80 N.
        if isPID
            ef.Fm.Value = 80.0;
        else
            ef.Fm.Value = 0.0;
        end
    end

    function setEnable(flds, on)
        for k = 1:numel(flds)
            if on
                flds{k}.Enable    = 'on';
                flds{k}.FontColor = WHITE;
            else
                flds{k}.Enable    = 'off';
                flds{k}.FontColor = [0.45 0.48 0.55];
            end
        end
    end

    %% ---- onReset ---------------------------------------------------------
    function onReset()
        ef.R.Value    = 0.50;   ef.barA.Value = 1e-4;
        ef.Ep.Value   = 70e9;   ef.Ej.Value   = 1e8;
        ef.mn.Value   = 10.0;   ef.Krot.Value = 50.0;
        % Open-Loop = pure actuation (no force, like the script); PID needs one.
        if contains(simDrop.Value,'PID'), ef.Fm.Value = 80.0; else, ef.Fm.Value = 0.0; end
        ef.tf.Value   = 2.0;
        ef.ttot.Value = 12.0;   ef.dt.Value   = 0.01;
        ef.alp.Value  = 0.10;   ef.bet.Value  = 0.00;
        ef.eact.Value = -0.15;  ef.tramp.Value= 1.5;
        ef.thold.Value= 1.0;
        ef.Kps.Value  = 0.6;    ef.Kis.Value  = 1.0;
        ef.Kds.Value  = 0.0;    ef.pls.Value  = 0.05;
        ef.tons.Value = 1.0;    ef.Kv.Value   = 0.0;
        ef.Kpr.Value  = 0.0;    ef.Kir.Value  = 0.0;
        ef.Kdr.Value  = 50;     ef.Mlim.Value = 0.0;
    end

    %% ---- onRun -----------------------------------------------------------
    % Collect parameters and run the solver synchronously on the main
    % thread. The solver reports progress through progressFcn, which calls
    % setProgress → drawnow, so the bar animates live during the run.
    function onRun()
        runBtn.Enable  = 'off';
        runBtn.Text    = '⏳ Running…';
        statusLbl.Text = 'Simulation running…';
        logReset({'Collecting parameters…'});
        setProgress(0,'starting');
        drawnow;

        % Pack every field value into a plain struct.
        p.simType   = simDrop.Value;
        p.baseDir   = baseDir;
        p.R         = ef.R.Value;
        p.barA      = ef.barA.Value;
        p.E_panel   = ef.Ep.Value;
        p.E_joint   = ef.Ej.Value;
        p.m_node    = ef.mn.Value;
        p.K_rot     = ef.Krot.Value;
        p.F_mag     = ef.Fm.Value;
        p.t_force   = ef.tf.Value;
        p.t_total   = ef.ttot.Value;
        p.dt        = ef.dt.Value;
        p.alpha_d   = ef.alp.Value;
        p.beta_d    = ef.bet.Value;
        p.strain_act= ef.eact.Value;
        p.t_ramp    = ef.tramp.Value;
        p.t_hold    = ef.thold.Value;
        p.Kp_s      = ef.Kps.Value;
        p.Ki_s      = ef.Kis.Value;
        p.Kd_s      = ef.Kds.Value;
        p.pLim_s    = ef.pls.Value;
        p.t_on_s    = ef.tons.Value;
        p.Kv_s      = ef.Kv.Value;
        p.Kp_rot    = ef.Kpr.Value;
        p.Ki_rot    = ef.Kir.Value;
        p.Kd_rot    = ef.Kdr.Value;
        p.M_lim_rot = ef.Mlim.Value;

        % Run on the main thread; @setProgress updates the bar live via the
        % solver's progressFcn. bg_hexCenterRing traps its own errors.
        r = bg_hexCenterRing(p, @setProgress);
        onSimDone(r);
    end

    %% ---- onSimDone (r = result struct returned by bg_hexCenterRing) ------
    function onSimDone(r)
        % Flush solver log to console box
        logAppend(r.log(:));

        if ~r.ok
            setProgress(0,'error');
            statusLbl.Text = 'ERROR — see console.';
            logAppend({['ERROR: ' r.err.message]});
            for k = 1:numel(r.err.stack)
                logAppend({sprintf('  @ %s  line %d', r.err.stack(k).name, ...
                                                      r.err.stack(k).line)});
            end
        else
            setProgress(1,'complete');
            statusLbl.Text = 'Simulation complete.';
            plotHexResults(r);

            % Enable the interactive 3-D viewer for this run's results.
            lastResult      = r;
            caseDrop.Items  = r.caseNames;
            caseDrop.Value  = r.caseNames{end};   % controlled case by default
            caseDrop.Enable = 'on';
            viewBtn.Enable  = 'on';
            exportBtn.Enable = 'on';
        end

        runBtn.Enable = 'on';
        runBtn.Text   = '▶  Run Simulation';
        drawnow;
    end

    %% ---- onView ----------------------------------------------------------
    % Launch the standalone interactive 3-D viewer (orbit / pan / zoom +
    % time slider + play) for the case selected in caseDrop.
    function onView()
        if isempty(lastResult) || ~lastResult.ok
            return;
        end
        r   = lastResult;
        idx = find(strcmp(r.caseNames, caseDrop.Value), 1);
        if isempty(idx), idx = numel(r.caseNames); end
        U = r.caseUhis{idx};

        % Guard against a diverged solve: non-finite displacements would blow
        % up the viewer's axis limits (empty box). Trim to the finite portion
        % and tell the user (usually a sign the PID gains are too aggressive).
        finPerFrame = all(isfinite(reshape(U, size(U,1), [])), 2);
        badFrame    = find(~finPerFrame, 1);
        if ~isempty(badFrame)
            keep = max(1, badFrame - 1);
            U = U(1:keep,:,:);
            logAppend({sprintf( ...
                ['! %s diverged at t = %.2f s — showing first %d frame(s). ' ...
                 'Try lower PID gains.'], caseDrop.Value, ...
                (badFrame-1)*r.dt_view, keep)});
        end
        if size(U,1) < 2
            logAppend({'! Not enough finite frames to visualise this case.'});
            return;
        end

        % Auto-magnify: deflections are tiny vs. the ring size, so scale them
        % to ~12% of the structure span for visibility. The viewer's Magnify
        % box lets the user override this.
        span = max(max(r.node_coords,[],1) - min(r.node_coords,[],1));
        maxD = max(abs(U(isfinite(U))));
        mag  = 1;
        if ~isempty(maxD) && maxD > 0, mag = max(1, 0.12*span/maxD); end

        logAppend({sprintf( ...
            'Viewer: %s — %d frames, %d nodes, max|U|=%.4g m, magnify ×%.0f', ...
            caseDrop.Value, size(U,1), size(U,2), maxD, mag)});
        drawnow;

        viz = struct();
        viz.node_coords = r.node_coords;
        viz.hex_nodes   = r.hex_nodes;
        viz.Uhis        = U;
        viz.bar_conn    = r.bar_conn;
        viz.joint_ids   = r.joint_ids;
        if strcmp(r.mode,'pid') && idx == numel(r.caseNames) && isfield(r,'act_loaded')
            viz.actuator_ids = r.act_loaded;
            viz.actuator_cmd = r.prestrain_his;
        elseif strcmp(r.mode,'openloop') && isfield(r,'act_ids')
            viz.actuator_ids = r.act_ids;
            viz.actuator_cmd = r.actRatio - 1;
        end
        viz.dt          = r.dt_view;
        viz.skip        = max(1, round(size(U,1)/300));  % ≤~300 smooth frames
        viz.magnify     = mag;
        % No magnify factor here — the viewer appends the live one to the title.
        viz.title       = sprintf('%s  —  %s', r.simType, caseDrop.Value);
        try
            Interactive_Deformation_Viewer(viz);
        catch ME
            msg = sprintf('%s', ME.message);
            if ~isempty(ME.stack)
                msg = sprintf('%s\n(@ %s line %d)', msg, ...
                              ME.stack(1).name, ME.stack(1).line);
            end
            uialert(fig, msg, '3D viewer error');   % loud, can't be missed
            logAppend({['3D viewer error: ' ME.message]});
        end
    end

    %% ---- onExport --------------------------------------------------------
    % Render the case selected in caseDrop to an MP4 (top-down X–Z animation,
    % same style as Hex_Center_Ring_CONTROL_RotSpr.m). Small force-response
    % motions are auto-magnified for visibility, like the 3-D viewer.
    function onExport()
        if isempty(lastResult) || ~lastResult.ok
            return;
        end
        r   = lastResult;
        idx = find(strcmp(r.caseNames, caseDrop.Value), 1);
        if isempty(idx), idx = numel(r.caseNames); end

        defName = sprintf('hex_ring_%s.mp4', ...
            lower(regexprep(caseDrop.Value, '\W+', '_')));
        [fname, fpath] = uiputfile('*.mp4', 'Export animation as MP4', defName);
        if isequal(fname, 0), return; end          % cancelled
        mp4File = fullfile(fpath, fname);

        % Trim any diverged (non-finite) tail and pick a magnify factor.
        U = r.caseUhis{idx};
        finPerFrame = all(isfinite(reshape(U, size(U,1), [])), 2);
        bad = find(~finPerFrame, 1);
        if ~isempty(bad), U = U(1:max(1,bad-1),:,:); end
        if size(U,1) < 2
            logAppend({'! Not enough finite frames to export this case.'});
            return;
        end
        span = max(max(r.node_coords,[],1) - min(r.node_coords,[],1));
        maxD = max(abs(U(isfinite(U))));
        mag  = 1;
        if ~isempty(maxD) && maxD > 0, mag = max(1, 0.12*span/maxD); end
        skip = max(1, round(size(U,1)/300));   % ≤~300 frames
        fps  = 20;

        exportBtn.Enable = 'off';  exportBtn.Text = '⏳ Exporting…';
        runBtn.Enable = 'off';     viewBtn.Enable = 'off';
        statusLbl.Text = 'Rendering MP4…';
        logAppend({sprintf('Exporting "%s" — %d frames, magnify ×%.0f…', ...
            caseDrop.Value, ceil(size(U,1)/skip), mag)});
        setProgress(0,'rendering MP4');  drawnow;

        try
            export_mp4(r, idx, U, mp4File, mag, skip, fps, @setProgress);
            setProgress(1,'MP4 saved');
            statusLbl.Text = 'MP4 saved.';
            logAppend({['Saved: ' mp4File]});
        catch ME
            setProgress(0,'export failed');
            statusLbl.Text = 'MP4 export failed — see console.';
            logAppend({['MP4 export error: ' ME.message]});
            if ~isempty(ME.stack)
                logAppend({sprintf('  @ %s line %d', ...
                    ME.stack(1).name, ME.stack(1).line)});
            end
        end
        exportBtn.Enable = 'on';  exportBtn.Text = '🎬  Export MP4';
        runBtn.Enable = 'on';     viewBtn.Enable = 'on';
        drawnow;
    end

    %% ---- setProgress -----------------------------------------------------
    function setProgress(frac, stage)
        frac = max(0, min(1, frac));
        w    = max(1, round(frac * PROG_W));
        progFill.Position = [0 0 w 14];
        if frac >= 1
            progFill.BackgroundColor = TEAL;
        else
            progFill.BackgroundColor = GREEN_BTN;
        end
        if nargin > 1 && ~isempty(stage)
            statusLbl.Text = sprintf('%3.0f%%  —  %s', frac*100, stage);
        end
        drawnow limitrate;
    end

    %% ---- plotHexResults --------------------------------------------------
    % Handles both modes:
    %   'pid'      — passive vs. strain-PID (ring 1 + center centroid X-disp)
    %   'openloop' — center + 6 ring centroid X-disp with prescribed actuation
    function plotHexResults(r)
        % Ring 4 is the clamped panel; it gets grey, but a light grey — the
        % dark grey used on the white MP4 canvas vanishes on these dark axes.
        ring_clrs = [1.00 0.82 0.10;   % ring 1 — gold (loaded)
                     0.30 0.55 0.80;
                     0.30 0.55 0.80;
                     0.72 0.74 0.78;   % ring 4 — grey (clamped)
                     0.30 0.55 0.80;
                     0.30 0.55 0.80];

        % ---- Displacement axes ----
        cla(ax_disp); hold(ax_disp,'on'); grid(ax_disp,'on');
        hD = gobjects(0);
        if strcmp(r.mode,'pid')
            hD(end+1) = plot(ax_disp, r.time, r.u_ring1_pass*1e3, 'Color',[0.85 0.25 0.25], ...
                'LineWidth',2.0,'DisplayName','Ring 1 — Passive');
            hD(end+1) = plot(ax_disp, r.time, r.u_ring_ctrl(:,1)*1e3, 'Color',[0.25 0.60 0.95], ...
                'LineWidth',2.0,'DisplayName','Ring 1 — Strain PID');
            hD(end+1) = plot(ax_disp, r.time, r.u_center_pass*1e3, 'Color',[0.85 0.45 0.25], ...
                'LineWidth',1.3,'LineStyle','--','DisplayName','Center — Passive');
            hD(end+1) = plot(ax_disp, r.time, r.u_center_ctrl*1e3, 'Color',[0.10 0.70 0.65], ...
                'LineWidth',1.3,'LineStyle','--','DisplayName','Center — PID');
            xline(ax_disp, r.t_ctrl_on,'--','Color',[0.80 0.80 0.80], ...
                'LineWidth',1.2,'Label','ctrl on','LabelOrientation','horizontal', ...
                'LabelVerticalAlignment','top','LabelHorizontalAlignment','right', ...
                'FontSize',9);
            title(ax_disp, sprintf(['Hex Center Ring PID — F=%.0f N | ' ...
                'Kp=%.2g Ki=%.2g Kd=%.2g'], r.F_mag, r.Kp, r.Ki, r.Kd), ...
                'Color',WHITE,'FontSize',12);
        else
            hD(end+1) = plot(ax_disp, r.time, r.u_center_ctrl*1e3, 'Color',[0.10 0.70 0.65], ...
                'LineWidth',1.8,'DisplayName','Center');
            for k = 1:size(r.u_ring_ctrl,2)
                hD(end+1) = plot(ax_disp, r.time, r.u_ring_ctrl(:,k)*1e3, ...
                    'Color',ring_clrs(k,:),'LineWidth',1.2, ...
                    'DisplayName',sprintf('Ring %d',k)); %#ok<AGROW>
            end
            title(ax_disp, sprintf('Hex Center Ring — Open Loop (actuator strain %.0f%%)', ...
                r.strain_act*100), 'Color',WHITE,'FontSize',12);
        end
        yline(ax_disp, 0,':','Color',[0.60 0.60 0.60],'LineWidth',0.8);
        xlabel(ax_disp,'Time (s)','Color',WHITE);
        ylabel(ax_disp,'Centroid X-disp (mm)','Color',WHITE);
        xlim(ax_disp,[r.time(1) r.time(end)]);
        % Legend built from the data handles only — passing them explicitly keeps
        % the xline/yline guides out of it (they'd show up as "data1", "data2").
        legend(ax_disp, hD, 'Location','best','TextColor',WHITE, ...
            'Color',[0.14 0.17 0.24],'EdgeColor',[0.30 0.33 0.42]);

        % ---- Actuator axes ----
        cla(ax_act); hold(ax_act,'on'); grid(ax_act,'on');
        if strcmp(r.mode,'pid')
            nb = size(r.prestrain_his,2);
            act_c = lines(max(nb,1));
            hA = gobjects(1,nb);
            for b = 1:nb
                hA(b) = plot(ax_act, r.time, r.prestrain_his(:,b)*100, ...
                    'Color',act_c(b,:),'LineWidth',1.2, ...
                    'DisplayName',sprintf('joint %d',b));
            end
            % Scale to the commands, not to the saturation limit: the limit is
            % often 10× the actual command and would flatten the traces onto
            % the zero line. Draw the limit guides only when they fit on-scale.
            yl  = padLimit(r.prestrain_his(:)*100);
            lim = r.pLim*100;
            ttl = sprintf('Loaded-side actuator commands (%d joint bars)', nb);
            if lim <= yl
                yl = max(yl, lim*1.15);
                yline(ax_act,  lim,'--','Color',[0.80 0.80 0.80], ...
                    'LineWidth',1.2,'Label','limit','FontSize',9, ...
                    'LabelHorizontalAlignment','left');
                yline(ax_act, -lim,'--','Color',[0.80 0.80 0.80],'LineWidth',1.2);
            else
                ttl = sprintf('%s — ±%.2g%% limit off-scale', ttl, lim);
            end
            ylim(ax_act, [-yl yl]);
            yline(ax_act, 0,':','Color',[0.60 0.60 0.60],'LineWidth',0.8);
            ylabel(ax_act,'Prestrain command (%)','Color',WHITE);
            title(ax_act, ttl, 'Color',WHITE,'FontSize',12);
        else
            nb = size(r.actRatio,2);
            act_c = lines(nb);
            hA = gobjects(1,nb);
            for b = 1:nb
                hA(b) = plot(ax_act, r.time, r.actRatio(:,b), ...
                    'Color',act_c(b,:),'LineWidth',1.0, ...
                    'DisplayName',sprintf('joint %d',b));
            end
            yline(ax_act, 1,'--','Color',[0.80 0.80 0.80], ...
                'LineWidth',1.2,'Label','natural','FontSize',9, ...
                'LabelHorizontalAlignment','left');
            yl = padLimit(r.actRatio(:) - 1);
            ylim(ax_act, 1 + [-yl yl]);
            ylabel(ax_act,'L_0(t)/L_{0,nat}','Color',WHITE);
            title(ax_act, sprintf('Actuator rest-length ratios (%d joint bars)', nb), ...
                'Color',WHITE,'FontSize',12);
        end
        xlabel(ax_act,'Time (s)','Color',WHITE);
        xlim(ax_act,[r.time(1) r.time(end)]);
        legend(ax_act, hA, 'Location','best','TextColor',WHITE, ...
            'Color',[0.14 0.17 0.24],'EdgeColor',[0.30 0.33 0.42],'NumColumns',2);

        % ---- Rotational-joint moment axes ----
        cla(ax_rot); hold(ax_rot,'on'); grid(ax_rot,'on');
        if strcmp(r.mode,'pid') && isfield(r,'rot_moment_his')
            M  = r.rot_moment_his;
            ns = size(M,2);
            % 24 hinges would make an unreadable legend; show the envelope and
            % the single hardest-working hinge instead.
            [~, iw] = max(max(abs(M),[],1));
            hR = gobjects(1,3);
            hR(1) = plot(ax_rot, r.time, max(M,[],2), 'Color',[0.30 0.55 0.80], ...
                'LineWidth',1.0,'DisplayName','max over hinges');
            hR(2) = plot(ax_rot, r.time, min(M,[],2), 'Color',[0.30 0.55 0.80], ...
                'LineWidth',1.0,'DisplayName','min over hinges');
            hR(3) = plot(ax_rot, r.time, M(:,iw), 'Color',[1.00 0.82 0.10], ...
                'LineWidth',1.8,'DisplayName',sprintf('hinge %d (peak)',iw));
            yl = padLimit(M(:));
            ylim(ax_rot, [-yl yl]);
            yline(ax_rot, 0,':','Color',[0.60 0.60 0.60],'LineWidth',0.8);
            xline(ax_rot, r.t_ctrl_on,'--','Color',[0.80 0.80 0.80], ...
                'LineWidth',1.2,'Label','ctrl on','LabelOrientation','horizontal', ...
                'LabelVerticalAlignment','top','LabelHorizontalAlignment','right', ...
                'FontSize',9);
            ylabel(ax_rot,'Applied hinge moment (N·m)','Color',WHITE);
            title(ax_rot, sprintf(['Rotational-joint PID moments (%d hinges) — ' ...
                'Kp=%.3g Ki=%.3g Kd=%.3g'], ns, r.Kp_rot, r.Ki_rot, r.Kd_rot), ...
                'Color',WHITE,'FontSize',12);
            legend(ax_rot, hR, 'Location','best','TextColor',WHITE, ...
                'Color',[0.14 0.17 0.24],'EdgeColor',[0.30 0.33 0.42]);
        else
            title(ax_rot,'Rotational-joint PID runs in PID Control mode only', ...
                'Color',WHITE,'FontSize',12);
        end
        xlabel(ax_rot,'Time (s)','Color',WHITE);
        xlim(ax_rot,[r.time(1) r.time(end)]);

        % ---- Console summary ----
        if strcmp(r.mode,'pid')
            logAppend({ ...
                sprintf('Ring1 passive SS : %+.3f mm', r.u_ring1_pass(end)*1e3); ...
                sprintf('Ring1 PID SS     : %+.3f mm', r.u_ring_ctrl(end,1)*1e3)});
            if abs(r.u_ring1_pass(end)) > 1e-9
                logAppend({sprintf('Reduction        : %.1f %%', ...
                    100*(1-abs(r.u_ring_ctrl(end,1))/abs(r.u_ring1_pass(end))))});
            end
            if isfield(r,'rot_error_his')
                logAppend({sprintf('Hinge |e| final  : %.3g rad  (peak M %.3g N·m)', ...
                    max(abs(r.rot_error_his(end,:))), max(abs(r.rot_moment_his(:))))});
            end
        end
    end

end  % Hex_Ring_PID_GUI

%% ---- Simulation driver (file-level) ---------------------------------------
% Runs synchronously on the main thread. Returns a plain struct r with
% results (and a trapped error in r.err / r.ok on failure), and reports
% progress through the progCb(frac,stage) callback passed in from the GUI.

function r = bg_hexCenterRing(p, progCb)
%BG_HEXCENTERRING  Drive the 1-center + 6-ring rot-spring hex model.
%   Open-Loop mode : one solve, prescribed rest-length trajectory (all 12
%                    joint actuators), matching Hex_Center_Ring_CONTROL_RotSpr.
%   PID mode       : passive solve + strain-PID solve, where the controller
%                    drives the LOADED-SIDE joint bars (touching ring hex 1)
%                    to zero strain, rejecting the +X force impulse.
    r.simType = p.simType;
    r.ok  = false;
    r.log = {};
    r.err = [];

    mkProg = @(base,span,stage) @(f) progCb(base + span*f, stage);

    try
        progCb(0.0,'building model');
        addpath(genpath(fullfile(p.baseDir,'00_SourceCode_Elements')));
        addpath(genpath(fullfile(p.baseDir,'00_SourceCode_Solver')));

        r.log{end+1} = 'Building 1+6 hex ring (rot-spring joints)…';
        m = build_hex_center_ring(p);
        r.log{end+1} = sprintf('  %d nodes, %d bars, %d joint actuators.', ...
            m.nNodes, m.nBars, numel(m.act_ids));

        totalSteps = round(p.t_total / p.dt);
        stepForce  = round(p.t_force / p.dt);

        % External +X force on ring-hex-1 outer nodes.
        nFN  = numel(m.outer_ring1);
        Fext = zeros(totalSteps, m.nNodes, 3);
        for n = m.outer_ring1(:)'
            Fext(1:stepForce, n, 1) = p.F_mag / nFN;
        end

        % Loaded-side joint bars: those with an endpoint on ring hex 1.
        panel1_nodes = m.hex_nodes(2,:);
        loaded = false(numel(m.act_ids),1);
        for a = 1:numel(m.act_ids)
            b = m.act_ids(a);
            if any(panel1_nodes == m.bar_conn(b,1)) || ...
               any(panel1_nodes == m.bar_conn(b,2))
                loaded(a) = true;
            end
        end
        act_loaded = m.act_ids(loaded);

        L0_nat_all = m.bar.L0_vec(m.act_ids);
        isPID = contains(p.simType,'PID');

        if ~isPID
            % ---- OPEN LOOP -------------------------------------------------
            nActBars   = numel(m.act_ids);
            ramp_steps = round(p.t_ramp / p.dt);
            hold_steps = round(p.t_hold / p.dt);
            cycle      = 2*ramp_steps + hold_steps;

            L0_his = repmat(L0_nat_all', totalSteps, 1);
            for b = 1:nActBars
                dL    = p.strain_act * L0_nat_all(b);
                shift = (b-1) * cycle;
                for s = 1:ramp_steps
                    idx = shift + s;
                    if idx <= totalSteps
                        L0_his(idx,b) = L0_nat_all(b) + dL*(s/ramp_steps);
                    end
                end
                for s = 1:hold_steps
                    idx = shift + ramp_steps + s;
                    if idx <= totalSteps
                        L0_his(idx,b) = L0_nat_all(b) + dL;
                    end
                end
                for s = 1:ramp_steps
                    idx = shift + ramp_steps + hold_steps + s;
                    if idx <= totalSteps
                        L0_his(idx,b) = L0_nat_all(b) + dL*(1 - s/ramp_steps);
                    end
                end
            end

            act_struct.bar_ids = m.act_ids;
            act_struct.L0_nat  = L0_nat_all;
            act_struct.L0_his  = L0_his;

            m.assembly.Initialize_Assembly();
            caa = Solver_CAA_Dynamics;
            caa.assembly = m.assembly;  caa.supp = m.supp;
            caa.dt = p.dt;  caa.Fext = Fext;
            caa.alpha = p.alpha_d;  caa.beta = p.beta_d;
            caa.rotSprTargetAngle = [];
            caa.actuation = act_struct;
            caa.progressFcn = mkProg(0.05, 0.95, 'open-loop actuation');
            r.log{end+1} = 'Running open-loop actuation…';
            Uctrl = caa.Solve();
            r.log{end+1} = '  Done.';

            r.mode       = 'openloop';
            r.strain_act = p.strain_act;
            r.actRatio   = L0_his ./ L0_nat_all';    % totalSteps × nActBars
            r.caseNames  = {'Open Loop'};
            r.caseUhis   = {Uctrl};
            Upass = [];
        else
            % ---- PID -------------------------------------------------------
            % Case 1: passive (force on, no control).
            m.assembly.Initialize_Assembly();
            caa = Solver_CAA_Dynamics;
            caa.assembly = m.assembly;  caa.supp = m.supp;
            caa.dt = p.dt;  caa.Fext = Fext;
            caa.alpha = p.alpha_d;  caa.beta = p.beta_d;
            caa.rotSprTargetAngle = [];
            caa.progressFcn = mkProg(0.05, 0.45, 'passive ring');
            r.log{end+1} = '[1/2] Passive ring…';
            Upass = caa.Solve();
            r.log{end+1} = '  Done.';

            % Case 2: strain-PID on the loaded-side joint bars.
            ctrl.bar_ids         = act_loaded;
            ctrl.Kp              = p.Kp_s;
            ctrl.Ki              = p.Ki_s;
            ctrl.Kd              = p.Kd_s;
            ctrl.target_strain   = 0;
            ctrl.prestrain_limit = p.pLim_s;
            ctrl.t_on            = p.t_on_s;
            % Filter the derivative term: the raw derivative drives the
            % actuators into bang-bang saturation chatter on this lightly-
            % constrained ring (the panels "glitch" late in the run).
            if isfield(p,'deriv_tau'), ctrl.deriv_tau = p.deriv_tau;
            else,                      ctrl.deriv_tau = 0.03; end
            % Active velocity damping: feed back each joint's axial strain
            % rate sensed from node velocity, modelled as a panel IMU + rate
            % gyro (rigid-body velocity per hexagon). This is a collocated,
            % dissipative damper that the strain-only PID cannot provide.
            if isfield(p,'Kv_s'), ctrl.Kv = p.Kv_s; else, ctrl.Kv = 0; end
            ctrl.vel_sensor = 'imu';
            pn = cell(1, m.nHex);
            for h = 1:m.nHex, pn{h} = m.hex_nodes(h,:); end
            ctrl.imu = struct('panel_nodes', {pn});

            m.assembly.Initialize_Assembly();
            caa2 = Solver_CAA_Dynamics;
            caa2.assembly = m.assembly;  caa2.supp = m.supp;
            caa2.dt = p.dt;  caa2.Fext = Fext;
            caa2.alpha = p.alpha_d;  caa2.beta = p.beta_d;
            caa2.rotSprTargetAngle = [];
            caa2.control = ctrl;
            % Full PID on the rotational joints (settles the ring's swing mode
            % and holds the hinge angles). Kd_rot alone == the old rotDamping.
            % sign=-1 matches the 3-node spring's moment convention (see the
            % rotControl property doc). Passive case gets none, for contrast.
            if any([p.Kp_rot, p.Ki_rot, p.Kd_rot] ~= 0)
                if p.M_lim_rot > 0, mlim = p.M_lim_rot; else, mlim = Inf; end
                caa2.rotControl = struct('Kp', p.Kp_rot, 'Ki', p.Ki_rot, ...
                    'Kd', p.Kd_rot, 't_on', p.t_on_s, 'sign', -1, ...
                    'moment_limit', mlim);
            end
            caa2.progressFcn = mkProg(0.50, 0.50, 'strain PID + joint PID ring');
            r.log{end+1} = sprintf(['[2/2] Controlled ring (%d joints; hinge PID ' ...
                'Kp=%.3g Ki=%.3g Kd=%.3g)…'], numel(act_loaded), ...
                p.Kp_rot, p.Ki_rot, p.Kd_rot);
            [Uctrl, ctrlLog] = caa2.Solve();
            r.log{end+1} = '  Done.';

            r.mode          = 'pid';
            r.prestrain_his = ctrlLog.prestrain_his;
            r.act_loaded    = act_loaded;
            if isfield(ctrlLog,'rot_dtheta_his')
                r.rot_dtheta_his = ctrlLog.rot_dtheta_his;
                r.rot_moment_his = ctrlLog.rot_moment_his;
                r.rot_error_his  = ctrlLog.rot_error_his;
            end
            r.Kp_rot        = p.Kp_rot;
            r.Ki_rot        = p.Ki_rot;
            r.Kd_rot        = p.Kd_rot;
            r.caseNames     = {'Passive','Controlled (strain + hinge PID)'};
            r.caseUhis      = {Upass, Uctrl};
        end

        % ---- Common results: center + ring centroid X-displacements -------
        r.time         = (0:totalSteps-1) * p.dt;
        r.u_center_ctrl = mean(squeeze(Uctrl(:, m.center_nodes, 1)), 2);
        r.u_ring_ctrl   = zeros(totalSteps, m.nRing);
        for k = 1:m.nRing
            r.u_ring_ctrl(:,k) = mean(squeeze(Uctrl(:, m.hex_nodes(k+1,:), 1)), 2);
        end
        if ~isempty(Upass)
            r.u_center_pass = mean(squeeze(Upass(:, m.center_nodes, 1)), 2);
            r.u_ring1_pass  = mean(squeeze(Upass(:, m.hex_nodes(2,:), 1)), 2);
        end

        r.nAct      = numel(m.act_ids);
        r.F_mag     = p.F_mag;
        r.Kp        = p.Kp_s;
        r.Ki        = p.Ki_s;
        r.Kd        = p.Kd_s;
        r.pLim      = p.pLim_s;
        r.t_ctrl_on = p.t_on_s;

        % 3D-viewer payload: geometry + full displacement history per case.
        r.node_coords = m.node_coords;
        r.hex_nodes   = m.hex_nodes;
        r.bar_conn    = m.bar_conn;
        r.joint_ids   = find(m.is_joint);
        r.act_ids     = m.act_ids;
        r.dt_view     = p.dt;

        r.ok = true;
    catch ME
        r.err = ME;
    end
end

%% ---- Model builder (file-level) -------------------------------------------
function m = build_hex_center_ring(p)
%BUILD_HEX_CENTER_RING  Geometry, bars, rot-springs and assembly for the
%   1-center + 6-ring hex model with 'triangle' joints and 3-node
%   rotational springs. Ported from Hex_Center_Ring_CONTROL_RotSpr.m.
    R      = p.R;
    nRing  = 6;
    nHex   = 1 + nRing;
    d_ring = 3 * R;

    % --- Geometry ---
    raw_coords = zeros(nHex*6, 3);
    raw_hex    = zeros(nHex, 6);
    for j = 1:6
        raw_coords(j,:) = [R*cos((j-1)*pi/3), 0, R*sin((j-1)*pi/3)];
        raw_hex(1,j) = j;
    end
    for k = 1:nRing
        theta_k = (k-1)*pi/3;
        cx = d_ring*cos(theta_k);  cz = d_ring*sin(theta_k);
        for j = 1:6
            phi_j = (theta_k + pi) + (j-1)*pi/3;
            idx   = 6 + (k-1)*6 + j;
            raw_coords(idx,:) = [cx + R*cos(phi_j), 0, cz + R*sin(phi_j)];
            raw_hex(k+1,j) = idx;
        end
    end

    tol = R * 1e-6;
    [node_coords, ~, ic] = uniquetol(raw_coords, tol, 'ByRows',true,'DataScale',1);
    hex_nodes = zeros(nHex,6);
    for h = 1:nHex
        for j = 1:6
            hex_nodes(h,j) = ic(raw_hex(h,j));
        end
    end
    center_nodes     = hex_nodes(1,:);
    inner_ring_nodes = hex_nodes(2:end,1);
    nNodes           = size(node_coords,1);

    % --- Bars (6 perimeter + 3 fan-diagonals per hex, + 12 joint bars) ---
    max_bars      = nHex*9 + 50;
    raw_bars      = zeros(max_bars,2);
    is_joint_flag = false(max_bars,1);
    nb = 0;
    for h = 1:nHex
        v = hex_nodes(h,:);
        for j = 1:6, nb = nb+1; raw_bars(nb,:) = [v(j), v(mod(j,6)+1)]; end
        for j = 2:4, nb = nb+1; raw_bars(nb,:) = [v(1), v(j+1)];       end
    end
    for k = 1:nRing                       % center-ring joints
        nb = nb+1; raw_bars(nb,:) = [center_nodes(k), inner_ring_nodes(k)];
        is_joint_flag(nb) = true;
    end
    for k = 1:nRing                       % ring-ring joints
        k2 = mod(k,nRing)+1;
        nb = nb+1; raw_bars(nb,:) = [hex_nodes(k+1,6), hex_nodes(k2+1,2)];
        is_joint_flag(nb) = true;
    end
    raw_bars_s = sort(raw_bars(1:nb,:), 2);
    flag_s     = is_joint_flag(1:nb);
    [bar_conn, ~, ic_bar] = unique(raw_bars_s, 'rows');
    nBars    = size(bar_conn,1);
    is_joint = false(nBars,1);
    for b = 1:nb
        if flag_s(b), is_joint(ic_bar(b)) = true; end
    end
    act_ids = find(is_joint);

    % --- 3-node rotational springs: one at each end of each joint bar ---
    next_vertex = @(h,v) hex_nodes(h, mod(v,6)+1);
    joint_ends  = zeros(2*nRing, 4);
    for k = 1:nRing
        joint_ends(k,:) = [1, k, k+1, 1];
    end
    for k = 1:nRing
        k2 = mod(k,nRing)+1;
        joint_ends(nRing+k,:) = [k+1, 6, k2+1, 2];
    end
    nJoints    = size(joint_ends,1);
    rotspr_ijk = zeros(2*nJoints, 3);
    for jt = 1:nJoints
        hA = joint_ends(jt,1); vA = joint_ends(jt,2);
        hB = joint_ends(jt,3); vB = joint_ends(jt,4);
        nodeA = hex_nodes(hA,vA);  nodeB = hex_nodes(hB,vB);
        refA  = next_vertex(hA,vA); refB = next_vertex(hB,vB);
        rotspr_ijk(2*jt-1,:) = [refA, nodeA, nodeB];
        rotspr_ijk(2*jt,  :) = [refB, nodeB, nodeA];
    end

    % --- Elements & assembly ---
    node = Elements_Nodes;
    node.coordinates_mat = node_coords;
    node.mass_vec        = p.m_node * ones(nNodes,1);

    bar = Vec_Elements_Bars;
    bar.node_ij_mat = bar_conn;
    bar.A_vec       = p.barA * ones(nBars,1);
    E_vec           = p.E_panel * ones(nBars,1);
    E_vec(is_joint) = p.E_joint;
    bar.E_vec       = E_vec;

    rot_spr_3N = Std_Elements_RotSprings_3N;
    rot_spr_3N.node_ijk_mat  = rotspr_ijk;
    rot_spr_3N.rot_spr_K_vec = p.K_rot * ones(size(rotspr_ijk,1),1);

    assembly = Assembly_Hex_Origami_RotSpr;
    assembly.node       = node;
    assembly.bar        = bar;
    assembly.rot_spr_3N = rot_spr_3N;
    assembly.Initialize_Assembly();

    % --- Supports: ring hex 4 (h=5) clamped in XZ; all nodes Y-fixed ---
    ring4_nodes = unique(hex_nodes(5,:));
    supp = zeros(nNodes,4);
    supp(:,1) = (1:nNodes)';
    supp(:,3) = 1;
    for n = ring4_nodes(:)'
        supp(n,2) = 1;  supp(n,4) = 1;
    end

    % --- Pack ---
    m.nRing            = nRing;
    m.nHex             = nHex;
    m.node_coords      = node_coords;
    m.hex_nodes        = hex_nodes;
    m.bar_conn         = bar_conn;
    m.is_joint         = is_joint;
    m.act_ids          = act_ids;
    m.center_nodes     = center_nodes;
    m.inner_ring_nodes = inner_ring_nodes;
    m.outer_ring1      = setdiff(hex_nodes(2,:), inner_ring_nodes(1));
    m.assembly         = assembly;
    m.bar              = bar;
    m.supp             = supp;
    m.nNodes           = nNodes;
    m.nBars            = nBars;
end

%% ---- MP4 exporter (file-level) --------------------------------------------
function export_mp4(r, idx, U, mp4File, mag, skip, fps, progCb)
%EXPORT_MP4  Render one case's deformation history to an MP4 file.
%   Top-down (X–Z plane) animation matching Hex_Center_Ring_CONTROL_RotSpr:
%   undeformed ghost panels, deformed panels colour-coded by hexagon, joint
%   bars highlighted. Displacements are scaled by `mag` for visibility.
    nc   = r.node_coords;
    hn   = r.hex_nodes;
    bc   = r.bar_conn;
    jids = r.joint_ids;
    nHex = size(hn,1);

    Usub    = U(1:skip:end, :, :);
    nFrames = size(Usub,1);
    frameIds = 1:skip:size(U,1);

    hasAct = false;
    if strcmp(r.mode,'pid') && idx == numel(r.caseNames) && isfield(r,'act_loaded')
        actIds = r.act_loaded(:);
        cmdSub = r.prestrain_his(frameIds,:);
        hasAct = true;
    elseif strcmp(r.mode,'openloop') && isfield(r,'act_ids')
        actIds = r.act_ids(:);
        cmdSub = r.actRatio(frameIds,:) - 1;
        hasAct = true;
    end
    if hasAct
        cmdScale = max(abs(cmdSub(:)));
        if cmdScale < eps, cmdScale = 1; end
    end

    % Panel colours: center=teal, ring 1=gold (loaded), ring 4=dark (clamped).
    hexFace = repmat([0.30 0.55 0.80], nHex, 1);
    if nHex>=1, hexFace(1,:) = [0.10 0.70 0.65]; end
    if nHex>=2, hexFace(2,:) = [1.00 0.82 0.10]; end
    if nHex>=5, hexFace(5,:) = [0.25 0.25 0.25]; end

    span = max(max(nc,[],1) - min(nc,[],1));
    pad  = 0.06*span;
    xLim = [min(nc(:,1))-pad, max(nc(:,1))+pad];
    zLim = [min(nc(:,3))-pad, max(nc(:,3))+pad];

    vid = VideoWriter(mp4File, 'MPEG-4');
    vid.FrameRate = fps;
    open(vid);
    % A dedicated (visible) figure — getframe captures reliably from it.
    figMP = figure('Color','white','Position',[80 80 900 850], ...
                   'Name','Exporting MP4…','NumberTitle','off');
    axMP  = axes(figMP);
    cleanup = onCleanup(@() cleanupExport(vid, figMP));

    for fi = 1:nFrames
        cla(axMP); hold(axMP,'on'); axis(axMP,'equal','off');
        xlim(axMP,xLim); ylim(axMP,zLim);
        deform = nc + mag*squeeze(Usub(fi,:,:));

        % Undeformed ghost.
        for h = 1:nHex
            v = nc(hn(h,:), [1 3]);
            patch(axMP, v(:,1), v(:,2), [0.93 0.93 0.93], ...
                'EdgeColor',[0.80 0.80 0.80], 'FaceAlpha',0.30, ...
                'LineStyle','--', 'LineWidth',0.5);
        end
        % Deformed panels.
        for h = 1:nHex
            v = deform(hn(h,:), [1 3]);
            patch(axMP, v(:,1), v(:,2), hexFace(h,:), ...
                'EdgeColor','k', 'LineWidth',1.5);
        end
        % Joint bars.
        for b = 1:numel(jids)
            n1 = bc(jids(b),1);  n2 = bc(jids(b),2);
            p1 = deform(n1,[1 3]);  p2 = deform(n2,[1 3]);
            plot(axMP, [p1(1) p2(1)], [p1(2) p2(2)], '-', ...
                'Color',[0.55 0.55 0.55], 'LineWidth',2.2);
        end

        % Actuated hinges / joint bars and their signed axial actuation.
        if hasAct
            maxArrow = 0.07 * span;
            for a = 1:numel(actIds)
                n1 = bc(actIds(a),1);  n2 = bc(actIds(a),2);
                p1 = deform(n1,[1 3]);  p2 = deform(n2,[1 3]);
                axisVec = p2 - p1;
                L = norm(axisVec);
                if L < eps
                    dir = [1 0];
                else
                    dir = axisVec / L;
                end
                c = cmdSub(fi,a);
                clr = [1.00 0.20 0.10];
                if c < -eps
                    clr = [0.95 0.00 0.80];       % contraction / tension
                    v1 =  dir;  v2 = -dir;
                elseif c > eps
                    clr = [0.00 0.45 1.00];       % extension / compression
                    v1 = -dir;  v2 =  dir;
                else
                    v1 = [0 0];  v2 = [0 0];
                end
                qLen = maxArrow * min(1, abs(c)/cmdScale);
                plot(axMP, [p1(1) p2(1)], [p1(2) p2(2)], '-', ...
                    'Color',clr, 'LineWidth',3.8);
                quiver(axMP, p1(1), p1(2), qLen*v1(1), qLen*v1(2), 0, ...
                    'Color',clr, 'LineWidth',1.2, 'MaxHeadSize',0.7);
                quiver(axMP, p2(1), p2(2), qLen*v2(1), qLen*v2(2), 0, ...
                    'Color',clr, 'LineWidth',1.2, 'MaxHeadSize',0.7);
            end
        end

        t = (fi-1)*skip*r.dt_view;
        title(axMP, sprintf('%s  —  t = %.2f s   (magnify ×%.0f)', ...
              r.caseNames{idx}, t, mag), 'FontSize',12);
        drawnow;
        writeVideo(vid, getframe(figMP));

        if ~isempty(progCb) && (mod(fi, max(1,round(nFrames/40)))==0 || fi==nFrames)
            progCb(fi/nFrames, 'rendering MP4');
        end
    end
    % cleanup (close video + figure) runs via onCleanup.
end

function cleanupExport(vid, figMP)
    try
        close(vid);
    catch
    end
    if isgraphics(figMP), close(figMP); end
end

%% ---- Local widget helpers (module-level) ----------------------------------
function ef = mkField(parent, label, val, x, y, w, h)
    FIELD_BG = [0.20 0.23 0.32];
    LABEL_FG = [0.78 0.82 0.92];
    uilabel(parent,'Text',label,'Position',[x y+h+2 w 16], ...
        'FontColor',LABEL_FG,'FontSize',9);
    ef = uieditfield(parent,'numeric','Value',val, ...
        'Position',[x y w h], ...
        'BackgroundColor',FIELD_BG,'FontColor',[1 1 1],'FontSize',11);
end

function h = mkLabel(parent, txt, x, y, w, hgt, sz, wt, clr)
    h = uilabel(parent,'Text',txt,'Position',[x y w hgt], ...
        'FontSize',sz,'FontWeight',wt,'FontColor',clr);
end

function yl = padLimit(v)
%PADLIMIT  Symmetric half-range that comfortably contains the finite data in v.
    v = v(isfinite(v));
    if isempty(v), yl = 1; return; end
    yl = max(abs(v)) * 1.30;
    if yl < 1e-9, yl = 1e-9; end
end

function colorTab(tab, clr)
    try
        tab.BackgroundColor = clr;
    catch
    end
end
