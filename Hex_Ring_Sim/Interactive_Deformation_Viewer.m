function Interactive_Deformation_Viewer(viz)
%% Interactive_Deformation_Viewer
% Interactive 3-D viewer for a hex-ring deformation history.
%
% The user can ORBIT, PAN and ZOOM the camera freely (drag = rotate,
% right-drag / pan tool = pan, scroll = zoom) while scrubbing through the
% simulation with a time slider or the Play/Pause button. This makes it easy
% to look at the joint deformation from any angle and watch how it evolves.
%
% Controls
%   • Drag in the axes ............ orbit the camera around the structure
%   • Scroll wheel ................ zoom in / out
%   • Pan / Rotate / Zoom toolbar . standard MATLAB axes interaction tools
%   • Time slider ................. scrub to any instant
%   • Play / Pause button ......... animate forward (loops)
%   • Reset View button ........... restore the default 3-D view
%   • Magnify box ................. scale the displacements for visibility
%
% Input struct viz:
%   node_coords  (N×3)   reference coordinates                 (required)
%   hex_nodes    (nHex×6) node ids per hexagon                 (required)
%   Uhis         (S×N×3) displacement history                  (required)
%   bar_conn     (nBars×2) bar connectivity                    (optional)
%   joint_ids    (nJ×1)  joint-bar indices into bar_conn       (optional)
%   jointForce   (S×nJ)  joint axial force history (for color) (optional)
%   actuator_ids (nA×1)  actuated joint-bar indices            (optional)
%   actuator_cmd (S×nA)  signed prestrain / actuation command  (optional)
%   dt           time step (s)                                 (default 1)
%   skip         frame subsample stride                        (default 1)
%   magnify      displacement magnification factor             (default 1)
%   title        figure title string                           (optional)

    %% ---- Unpack / defaults ------------------------------------------------
    assert(all(isfield(viz, {'node_coords','hex_nodes','Uhis'})), ...
        'viz must contain node_coords, hex_nodes and Uhis.');
    node_coords = viz.node_coords;
    hex_nodes   = viz.hex_nodes;
    Uall        = viz.Uhis;
    dt      = getdef(viz,'dt',1);
    skip    = max(1, round(getdef(viz,'skip',1)));
    magnify = getdef(viz,'magnify',1);
    ttl     = getdef(viz,'title','Hex-ring deformation — interactive viewer');

    hasBars   = isfield(viz,'bar_conn')  && ~isempty(viz.bar_conn) && ...
                isfield(viz,'joint_ids') && ~isempty(viz.joint_ids);
    hasForce  = hasBars && isfield(viz,'jointForce') && ~isempty(viz.jointForce);
    if hasBars,  bar_conn = viz.bar_conn;  joint_ids = viz.joint_ids(:);  end
    hasAct = hasBars && isfield(viz,'actuator_ids') && ~isempty(viz.actuator_ids);
    if hasAct
        actuator_ids = viz.actuator_ids(:);
        hasActCmd = isfield(viz,'actuator_cmd') && ~isempty(viz.actuator_cmd);
    else
        hasActCmd = false;
    end

    % Subsample frames for smooth interaction
    fr      = 1:skip:size(Uall,1);
    Uhis    = Uall(fr,:,:);
    nFrames = size(Uhis,1);
    times   = (fr-1)*dt;
    nHex    = size(hex_nodes,1);

    if hasForce
        jF      = viz.jointForce(fr,:);
        Fscale  = max(abs(viz.jointForce(:)));  if Fscale < eps, Fscale = 1; end
        jcmap   = [0.15 0.35 1.0; 0.85 0.85 0.85; 1 0.15 0.15];  % comp-neutral-tens
        jpts    = [-Fscale, 0, Fscale];
    end
    if hasActCmd
        actCmd = viz.actuator_cmd(fr,:);
        cmdScale = max(abs(viz.actuator_cmd(:)));
        if cmdScale < eps, cmdScale = 1; end
    elseif hasAct
        cmdScale = 1;
    end

    %% ---- Fixed axis limits, framed on the UNDEFORMED structure ------------
    % Deliberately independent of the deformation magnitude: an unstable /
    % blown-up case (large finite or non-finite Uhis) would otherwise make
    % the limits huge and shrink the real structure to an invisible speck.
    % Here the body is always framed; extreme motion simply moves off-screen.
    coordMn = min(node_coords,[],1)';
    coordMx = max(node_coords,[],1)';
    ctr  = (coordMn + coordMx)/2;
    span = max(coordMx - coordMn);
    % In-plane (X,Z) gets a modest margin. The structure is essentially flat in
    % Y, so giving Y the same half-width as X and Z would frame a big empty
    % cube and shrink the model to a chip in the middle of it; Y gets a slab
    % deep enough for the magnified out-of-plane motion instead.
    halfXZ = max(span/2, 1e-3) * 1.15;
    halfY  = max(halfXZ * 0.25, 1e-3);
    half   = [halfXZ; halfY; halfXZ];
    Lim    = [ctr - half, ctr + half];
    arrowOff  = max(1e-5, 0.022 * span);  % sideways offset of the axial arrows
    ghostLift = max(1e-6, 0.002 * span);  % just enough to beat z-fighting

    %% ---- Panel colors -----------------------------------------------------
    hex_face        = repmat([0.30 0.55 0.80], nHex, 1);
    if nHex >= 1, hex_face(1,:) = [0.10 0.70 0.65]; end   % center — teal
    if nHex >= 2, hex_face(2,:) = [1.00 0.82 0.10]; end   % ring 1 — gold (loaded)
    if nHex >= 5, hex_face(5,:) = [0.25 0.25 0.25]; end   % ring 4 — dark (clamped)

    %% ---- Figure & axes ----------------------------------------------------
    fig = figure('color','white','Name',ttl,'NumberTitle','off', ...
                 'Position',[100 100 980 820]);
    ax  = axes('Parent',fig,'Position',[0.07 0.20 0.88 0.74]);
    hold(ax,'on');  grid(ax,'on');  box(ax,'on');  axis(ax,'equal');
    xlim(ax,Lim(1,:));  ylim(ax,Lim(2,:));  zlim(ax,Lim(3,:));
    xlabel(ax,'X (m)');  ylabel(ax,'Y — out of plane (m)');  zlabel(ax,'Z (m)');
    defView = [35 22];  view(ax,defView);
    try, enableDefaultInteractivity(ax); catch, end   % drag-orbit / scroll-zoom

    % Undeformed ghost outlines (static). Pushed one lift behind the deformed
    % panels along Y: drawn coplanar they z-fight, which streaks the panels
    % with white slivers wherever the deformation is small.
    for h = 1:nHex
        v = node_coords(hex_nodes(h,:),:);
        hg = patch('Parent',ax,'XData',v(:,1),'YData',v(:,2)-ghostLift,'ZData',v(:,3), ...
              'FaceColor',[0.92 0.92 0.92],'FaceAlpha',0.10, ...
              'EdgeColor',[0.8 0.8 0.8],'LineStyle','--','LineWidth',0.4);
        hideFromLegend(hg);
    end

    % Deformed panels (handles updated each frame)
    hPanel = gobjects(nHex,1);
    for h = 1:nHex
        v = node_coords(hex_nodes(h,:),:);
        hPanel(h) = patch('Parent',ax,'XData',v(:,1),'YData',v(:,2),'ZData',v(:,3), ...
              'FaceColor',hex_face(h,:),'FaceAlpha',0.95,'EdgeColor','k','LineWidth',1.2);
    end
    hideFromLegend(hPanel);

    % Joint bars (handles updated each frame)
    if hasBars
        nJ = numel(joint_ids);
        hJoint = gobjects(nJ,1);
        for b = 1:nJ
            n12 = bar_conn(joint_ids(b),:);
            p1 = node_coords(n12(1),:);  p2 = node_coords(n12(2),:);
            hJoint(b) = plot3(ax,[p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)], ...
                              '-','LineWidth',4.0,'Color',[0.6 0.6 0.6]);
        end
        hideFromLegend(hJoint);
    end

    % Actuated hinges / joint bars: bright overlay plus paired axial arrows.
    % setFrame recolours both by the sign of the command, so the legend keys
    % must be the sign colours — not the creation colour.
    ACT_IDLE   = [1.00 0.20 0.10];
    ACT_CONTRA = [0.95 0.00 0.80];
    ACT_EXTEND = [0.00 0.45 1.00];
    % A contracting actuator's arrows point inward, i.e. straight along the bar
    % they sit on. Same colour as the bar they would be invisible, so they get
    % their own dark colour and ride one lift above it.
    ARROW_C    = [0.15 0.15 0.15];
    if hasAct
        nA = numel(actuator_ids);
        % An actuated bar IS a joint bar. Hide the grey one underneath rather
        % than drawing the coloured overlay on top of it: coincident lines
        % z-fight, and nudging the overlay clear of them along Y just renders
        % it as a second line running alongside at every orbit angle.
        if hasBars
            [tfA, locA] = ismember(actuator_ids, joint_ids);
            set(hJoint(locA(tfA)), 'Visible','off');
        end
        hActLine = gobjects(nA,1);
        hActQ1   = gobjects(nA,1);
        hActQ2   = gobjects(nA,1);
        for a = 1:nA
            n12 = bar_conn(actuator_ids(a),:);
            p1 = node_coords(n12(1),:);  p2 = node_coords(n12(2),:);
            hActLine(a) = plot3(ax,[p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)], ...
                                '-','LineWidth',4.8,'Color',ACT_IDLE);
            % quiver3 renders the shaft but drops the arrowhead at these
            % scales, which leaves the actuation direction unreadable. Each
            % arrow is a plain polyline: shaft plus two barbs (see arrowPoly).
            hActQ1(a) = plot3(ax,nan,nan,nan,'-','Color',ARROW_C,'LineWidth',1.6);
            hActQ2(a) = plot3(ax,nan,nan,nan,'-','Color',ARROW_C,'LineWidth',1.6);
        end
        hideFromLegend(hActLine);
        hideFromLegend(hActQ1);
        hideFromLegend(hActQ2);
        % Off-screen proxies carry the whole legend — every real patch, bar and
        % arrow is excluded above, otherwise they enumerate as data1..dataN.
        % A quiver with NaN data draws no legend icon, so the arrow key is a
        % line with an arrowhead marker instead.
        hKey    = gobjects(4,1);
        hKey(1) = plot3(ax,nan,nan,nan,'-','LineWidth',4.8,'Color',ACT_CONTRA, ...
              'DisplayName','Actuator contracting');
        hKey(2) = plot3(ax,nan,nan,nan,'-','LineWidth',4.8,'Color',ACT_EXTEND, ...
              'DisplayName','Actuator extending');
        hKey(3) = plot3(ax,nan,nan,nan,'-','LineWidth',4.8,'Color',ACT_IDLE, ...
              'DisplayName','Actuator idle');
        hKey(4) = plot3(ax,nan,nan,nan,'-','LineWidth',1.6,'Color',ARROW_C, ...
              'Marker','>','MarkerSize',5,'MarkerFaceColor',ARROW_C, ...
              'DisplayName','Axial actuation direction');
        legend(ax, hKey, 'Location','northeast','AutoUpdate','off','FontSize',9);
    end

    hTitle = title(ax,'','FontSize',12);

    %% ---- UI controls ------------------------------------------------------
    uicontrol(fig,'Style','text','Units','normalized','FontSize',9, ...
        'BackgroundColor','white','HorizontalAlignment','left', ...
        'Position',[0.07 0.105 0.30 0.03], ...
        'String','Drag = orbit   •   scroll = zoom   •   pan/zoom toolbar');

    hSlider = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Min',1,'Max',nFrames,'Value',1, ...
        'SliderStep',[1/max(1,nFrames-1), max(2,round(nFrames/20))/max(1,nFrames-1)], ...
        'Position',[0.07 0.05 0.66 0.035]);

    hPlay = uicontrol(fig,'Style','togglebutton','Units','normalized', ...
        'String','Play','FontSize',10, ...
        'Position',[0.75 0.05 0.09 0.04],'Callback',@onPlay);

    uicontrol(fig,'Style','pushbutton','Units','normalized', ...
        'String','Reset View','FontSize',9, ...
        'Position',[0.85 0.05 0.10 0.04],'Callback',@(~,~)view(ax,defView));

    uicontrol(fig,'Style','text','Units','normalized','FontSize',9, ...
        'BackgroundColor','white','HorizontalAlignment','right', ...
        'Position',[0.74 0.105 0.10 0.03],'String','Magnify ×');
    hMag = uicontrol(fig,'Style','edit','Units','normalized','FontSize',9, ...
        'String',sprintf('%.4g',magnify),'Position',[0.85 0.105 0.10 0.03], ...
        'Callback',@onMag);

    % Live slider scrubbing
    addlistener(hSlider,'ContinuousValueChange',@(s,~)setFrame(round(get(s,'Value'))));

    %% ---- Playback timer ---------------------------------------------------
    frame = 1;
    tmr = timer('ExecutionMode','fixedRate', ...
                'Period', max(0.03, round((dt*skip)*1000)/1000), ...
                'TimerFcn',@onTick);
    fig.CloseRequestFcn = @onClose;

    setFrame(1);

    %% ===================== nested functions ===============================
    function onTick(~,~)
        f = frame + 1;  if f > nFrames, f = 1; end
        setFrame(f);
        set(hSlider,'Value',frame);
    end

    function onPlay(src,~)
        if get(src,'Value')
            set(src,'String','Pause');
            if strcmp(tmr.Running,'off'), start(tmr); end
        else
            set(src,'String','Play');
            if strcmp(tmr.Running,'on'), stop(tmr); end
        end
    end

    function onMag(src,~)
        v = str2double(get(src,'String'));
        if ~isnan(v) && v > 0
            magnify = v;
            % limits stay fixed (computed for the original magnify); just redraw
            setFrame(frame);
        else
            set(src,'String',sprintf('%.4g',magnify));
        end
    end

    function setFrame(k)
        k = max(1, min(nFrames, k));
        frame = k;
        U = squeeze(Uhis(k,:,:));
        def = node_coords + magnify*U;
        for hh = 1:nHex
            vv = def(hex_nodes(hh,:),:);
            set(hPanel(hh),'XData',vv(:,1),'YData',vv(:,2),'ZData',vv(:,3));
        end
        if hasBars
            if hasForce
                fcl = interp1(jpts, jcmap, max(jpts(1), min(jpts(end), jF(k,:))));
            end
            for bb = 1:numel(joint_ids)
                n12 = bar_conn(joint_ids(bb),:);
                p1 = def(n12(1),:);  p2 = def(n12(2),:);
                set(hJoint(bb),'XData',[p1(1) p2(1)],'YData',[p1(2) p2(2)], ...
                               'ZData',[p1(3) p2(3)]);
                if hasForce, set(hJoint(bb),'Color',fcl(bb,:)); end
            end
        end
        if hasAct
            cmd = zeros(1,numel(actuator_ids));
            if hasActCmd, cmd = actCmd(k,:); end
            maxArrow = 0.07 * span;
            for aa = 1:numel(actuator_ids)
                n12 = bar_conn(actuator_ids(aa),:);
                p1 = def(n12(1),:);  p2 = def(n12(2),:);
                axisVec = p2 - p1;
                L = norm(axisVec);
                if L < eps
                    dir = [1 0 0];
                else
                    dir = axisVec / L;
                end
                % The arrows run along the bar, so drawn on it they vanish
                % underneath. Step them sideways, in the structure's plane.
                sideVec = cross(dir, [0 1 0]);
                if norm(sideVec) < eps, sideVec = [0 0 1]; end
                sideUnit = sideVec / norm(sideVec);
                p1a = p1 + arrowOff*sideUnit;  p2a = p2 + arrowOff*sideUnit;
                c = cmd(aa);
                color = ACT_IDLE;
                if c < -eps
                    color = ACT_CONTRA;             % contraction / tension
                    v1 =  dir;  v2 = -dir;
                elseif c > eps
                    color = ACT_EXTEND;             % extension / compression
                    v1 = -dir;  v2 =  dir;
                else
                    v1 = [0 0 0];  v2 = [0 0 0];
                end
                magCmd = min(1, abs(c)/cmdScale);
                qLen = maxArrow * magCmd;
                set(hActLine(aa),'XData',[p1(1) p2(1)], ...
                                  'YData',[p1(2) p2(2)], ...
                                  'ZData',[p1(3) p2(3)], ...
                                  'Color',color);
                [aX,aY,aZ] = arrowPoly(p1a, qLen*v1, sideUnit);
                set(hActQ1(aa),'XData',aX,'YData',aY,'ZData',aZ,'Color',ARROW_C);
                [aX,aY,aZ] = arrowPoly(p2a, qLen*v2, sideUnit);
                set(hActQ2(aa),'XData',aX,'YData',aY,'ZData',aZ,'Color',ARROW_C);
            end
        end
        % The magnify factor belongs here (it is live-editable) and nowhere
        % else — callers must not bake it into ttl or it prints twice.
        set(hTitle,'String',sprintf('%s\n t = %.2f s   (frame %d / %d, magnify ×%.4g)', ...
                                    ttl, times(k), k, nFrames, magnify));
        drawnow limitrate;
    end

    function onClose(~,~)
        try, if strcmp(tmr.Running,'on'), stop(tmr); end; catch, end
        try, delete(tmr); catch, end
        delete(fig);
    end
end

function v = getdef(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end

function [X,Y,Z] = arrowPoly(base, vec, sideUnit)
%ARROWPOLY  Shaft-plus-barbs polyline for one arrow, NaN-separated.
%   A zero-length vec yields all-NaN, i.e. nothing is drawn (idle actuator).
    L = norm(vec);
    if L < eps
        X = nan; Y = nan; Z = nan;  return;
    end
    d   = vec / L;
    tip = base + vec;
    b1  = tip - 0.32*L*d + 0.20*L*sideUnit;
    b2  = tip - 0.32*L*d - 0.20*L*sideUnit;
    P   = [base; tip; nan(1,3); b1; tip; b2];
    X = P(:,1).';  Y = P(:,2).';  Z = P(:,3).';
end

function hideFromLegend(h)
%HIDEFROMLEGEND  Keep graphics objects out of any legend on their axes.
    for k = 1:numel(h)
        if isgraphics(h(k))
            h(k).Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    end
end
