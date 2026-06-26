function model = Build_Hex_Ring_Model(opts)
%% Build_Hex_Ring_Model
% Reusable builder for the 1-center + 6-ring hexagonal solar-panel array used
% by the hex-ring demos. Panels are stiff triangulated hexagons; panel-to-
% panel connections are compliant *bar joints* across each gap.
%
% opts (all optional; struct):
%   R          hexagon circumradius (m)            default 0.50
%   barA       bar cross-section area (m^2)         default 1e-4
%   E_panel    panel Young's modulus (Pa)           default 70e9
%   E_joint    joint Young's modulus (Pa)           default 1e8
%   m_node     lumped mass per node (kg)            default 10.0
%   joint_type 'triangle' (1 bar/gap) | 'Y' (hub+2) default 'triangle'
%
% Returns model struct with fields:
%   assembly         initialized Assembly_Hex_Origami (node + bar)
%   node_coords      (N×3) reference coordinates
%   hex_nodes        (nHex×6) node ids per hexagon
%   bar_conn         (nBars×2) bar connectivity
%   is_joint         (nBars×1) logical, true for joint bars
%   joint_ids        column of joint-bar indices into bar_conn
%   L0_joint         natural lengths of the joint bars
%   center_nodes, inner_ring_nodes
%   outer_ring1      outer nodes of ring panel 1 (load application set)
%   ring4_nodes      nodes of ring panel 4 (clamp set)
%   use_hub, cr_hub, rr_hub   (hub ids when joint_type = 'Y')
%   E_joint, barA, joint_type, R, nHex, nRing

    if nargin < 1, opts = struct(); end
    R          = getdef(opts,'R',0.50);
    barA       = getdef(opts,'barA',1e-4);
    E_panel    = getdef(opts,'E_panel',70e9);
    E_joint    = getdef(opts,'E_joint',1e8);
    m_node     = getdef(opts,'m_node',10.0);
    joint_type = getdef(opts,'joint_type','triangle');

    nRing   = 6;
    nHex    = 1 + nRing;
    d_ring  = 3 * R;
    use_hub = strcmp(joint_type, 'Y');

    %% --- Geometry ----------------------------------------------------------
    n_panel_raw = nHex * 6;
    max_raw     = n_panel_raw + 12 + 5;
    raw_coords  = zeros(max_raw, 3);
    raw_hex     = zeros(nHex, 6);

    for j = 1:6                                   % center hex (XZ plane)
        raw_coords(j, :) = [R*cos((j-1)*pi/3), 0, R*sin((j-1)*pi/3)];
        raw_hex(1, j) = j;
    end
    for k = 1:nRing                               % ring hexes
        theta_k = (k-1) * pi/3;
        cx = d_ring * cos(theta_k);  cz = d_ring * sin(theta_k);
        for j = 1:6
            phi_j = (theta_k + pi) + (j-1)*pi/3;
            idx   = 6 + (k-1)*6 + j;
            raw_coords(idx, :) = [cx + R*cos(phi_j), 0, cz + R*sin(phi_j)];
            raw_hex(k+1, j) = idx;
        end
    end
    n_placed = n_panel_raw;

    cr_hub_raw = zeros(nRing,1);  rr_hub_raw = zeros(nRing,1);
    if use_hub
        for k = 1:nRing
            theta_k = (k-1)*pi/3;
            n_placed = n_placed + 1;  cr_hub_raw(k) = n_placed;
            raw_coords(n_placed,:) = [1.5*R*cos(theta_k), 0, 1.5*R*sin(theta_k)];
        end
        for k = 1:nRing
            k2 = mod(k, nRing) + 1;
            p1 = raw_coords(raw_hex(k+1,6),:);  p2 = raw_coords(raw_hex(k2+1,2),:);
            n_placed = n_placed + 1;  rr_hub_raw(k) = n_placed;
            raw_coords(n_placed,:) = (p1 + p2) / 2;
        end
    end
    n_raw = n_placed;

    tol = R * 1e-6;
    [node_coords, ~, ic] = uniquetol(raw_coords(1:n_raw,:), tol, ...
                                     'ByRows', true, 'DataScale', 1);
    hex_nodes = zeros(nHex, 6);
    for h = 1:nHex
        for j = 1:6, hex_nodes(h,j) = ic(raw_hex(h,j)); end
    end
    center_nodes     = hex_nodes(1,:);
    inner_ring_nodes = hex_nodes(2:end,1);
    if use_hub, cr_hub = ic(cr_hub_raw);  rr_hub = ic(rr_hub_raw);
    else,       cr_hub = [];  rr_hub = [];  end
    nNodes = size(node_coords, 1);

    %% --- Bar connectivity --------------------------------------------------
    max_bars = nHex*9 + 50;
    raw_bars = zeros(max_bars, 2);
    is_joint_flag = false(max_bars, 1);
    nb = 0;
    for h = 1:nHex
        v = hex_nodes(h,:);
        for j = 1:6, nb=nb+1; raw_bars(nb,:) = [v(j), v(mod(j,6)+1)]; end
        for j = 2:4, nb=nb+1; raw_bars(nb,:) = [v(1), v(j+1)]; end
    end
    if use_hub
        for k = 1:nRing
            nb=nb+1; raw_bars(nb,:)=[center_nodes(k), cr_hub(k)];      is_joint_flag(nb)=true;
            nb=nb+1; raw_bars(nb,:)=[cr_hub(k), inner_ring_nodes(k)];  is_joint_flag(nb)=true;
        end
        for k = 1:nRing
            k2 = mod(k, nRing) + 1;
            nb=nb+1; raw_bars(nb,:)=[hex_nodes(k+1,6), rr_hub(k)];     is_joint_flag(nb)=true;
            nb=nb+1; raw_bars(nb,:)=[rr_hub(k), hex_nodes(k2+1,2)];    is_joint_flag(nb)=true;
        end
    else
        for k = 1:nRing
            nb=nb+1; raw_bars(nb,:)=[center_nodes(k), inner_ring_nodes(k)]; is_joint_flag(nb)=true;
        end
        for k = 1:nRing
            k2 = mod(k, nRing) + 1;
            nb=nb+1; raw_bars(nb,:)=[hex_nodes(k+1,6), hex_nodes(k2+1,2)]; is_joint_flag(nb)=true;
        end
    end
    raw_bars_s = sort(raw_bars(1:nb,:), 2);
    flag_s     = is_joint_flag(1:nb);
    [bar_conn, ~, ic_bar] = unique(raw_bars_s, 'rows');
    nBars = size(bar_conn, 1);
    is_joint = false(nBars, 1);
    for b = 1:nb
        if flag_s(b), is_joint(ic_bar(b)) = true; end
    end
    joint_ids = find(is_joint);

    %% --- Elements & assembly ----------------------------------------------
    node = Elements_Nodes;
    node.coordinates_mat = node_coords;
    node.mass_vec        = m_node * ones(nNodes, 1);

    bar = Vec_Elements_Bars;
    bar.node_ij_mat = bar_conn;
    bar.A_vec       = barA * ones(nBars, 1);
    E_vec           = E_panel * ones(nBars, 1);
    E_vec(is_joint) = E_joint;
    bar.E_vec       = E_vec;

    assembly = Assembly_Hex_Origami;
    assembly.node = node;
    assembly.bar  = bar;
    assembly.Initialize_Assembly();

    %% --- Convenience node sets --------------------------------------------
    outer_ring1 = setdiff(hex_nodes(2,:), inner_ring_nodes(1));
    ring4_nodes = unique(hex_nodes(5,:));   % ring hex 4 = h=5

    %% --- Pack model -------------------------------------------------------
    model = struct();
    model.assembly         = assembly;
    model.node_coords      = node_coords;
    model.hex_nodes        = hex_nodes;
    model.bar_conn         = bar_conn;
    model.is_joint         = is_joint;
    model.joint_ids        = joint_ids;
    model.L0_joint         = bar.L0_vec(joint_ids);
    model.center_nodes     = center_nodes;
    model.inner_ring_nodes = inner_ring_nodes;
    model.outer_ring1      = outer_ring1;
    model.ring4_nodes      = ring4_nodes;
    model.use_hub          = use_hub;
    model.cr_hub           = cr_hub;
    model.rr_hub           = rr_hub;
    model.E_joint          = E_joint;
    model.barA             = barA;
    model.joint_type       = joint_type;
    model.R                = R;
    model.nHex             = nHex;
    model.nRing            = nRing;
end

function v = getdef(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end
