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

    %% ---- Precompute fixed axis limits (over all frames, magnified) --------
    P = node_coords;  P = reshape(P,[1 size(P)]);
    allPos = P + magnify*Uhis;                       % (nFrames×N×3)
    mn = squeeze(min(min(allPos,[],1),[],2));
    mx = squeeze(max(max(allPos,[],1),[],2));
    ctr = (mn+mx)/2;  half = max((mx-mn)/2);  half = max(half, 1e-3)*1.15;
    Lim = [ctr-half, ctr+half];

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

    % Undeformed ghost outlines (static)
    for h = 1:nHex
        v = node_coords(hex_nodes(h,:),:);
        patch('Parent',ax,'XData',v(:,1),'YData',v(:,2),'ZData',v(:,3), ...
              'FaceColor',[0.92 0.92 0.92],'FaceAlpha',0.10, ...
              'EdgeColor',[0.8 0.8 0.8],'LineStyle','--','LineWidth',0.4);
    end

    % Deformed panels (handles updated each frame)
    hPanel = gobjects(nHex,1);
    for h = 1:nHex
        v = node_coords(hex_nodes(h,:),:);
        hPanel(h) = patch('Parent',ax,'XData',v(:,1),'YData',v(:,2),'ZData',v(:,3), ...
              'FaceColor',hex_face(h,:),'FaceAlpha',0.95,'EdgeColor','k','LineWidth',1.2);
    end

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
        'String',num2str(magnify),'Position',[0.85 0.105 0.10 0.03], ...
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
            set(src,'String',num2str(magnify));
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
        set(hTitle,'String',sprintf('%s\n t = %.2f s   (frame %d / %d, magnify ×%g)', ...
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
