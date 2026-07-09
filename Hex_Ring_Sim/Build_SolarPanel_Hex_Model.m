function model = Build_SolarPanel_Hex_Model(opts)
%% Build_SolarPanel_Hex_Model
%
% Builds a 1-center + 6-ring hexagonal solar-panel array with BENDING
% STIFFNESS. Extends Build_Hex_Ring_Model by adding 4-node rotational
% springs at the interior triangulation edges of every panel so that
% out-of-plane (Y) panel bending is physically resisted. The assembly
% returned is Assembly_SolarPanel_Hex (bars + rot springs).
%
% opts (all optional; struct or name-value):
%   R           hexagon circumradius (m)               default 0.50
%   barA        bar cross-section area (m^2)            default 1e-4
%   E_panel     Young's modulus of panel material (Pa)  default 70e9
%   E_joint     joint Young's modulus (Pa)              default 1e8
%   t_panel     panel thickness (m)                     default 3e-3
%   nu_panel    Poisson's ratio of panel material        default 0.33
%   m_node      lumped mass per node (kg)               default 10.0
%   joint_type  'triangle' | 'Y'                        default 'triangle'
%   panel_shear_factor
%               transverse panel-bar stiffness as a fraction of EA/L
%                                                            default 1e-4
%   joint_shear_factor
%               transverse joint stiffness as a fraction of EA/L
%                                                            default 0.25
%
% Returns model struct with all fields from Build_Hex_Ring_Model, plus:
%   assembly        Assembly_SolarPanel_Hex (bar + rot_spr_4N)
%   rot_spr_ijkl    (nSpr×4) node connectivity of bending springs
%   rot_spr_K       per-spring bending stiffness (N·m/rad)
%   t_panel, nu_panel, D_panel   (plate bending rigidity)

    if nargin < 1, opts = struct(); end
    R          = getdef(opts,'R',         0.50);
    barA       = getdef(opts,'barA',      1e-4);
    E_panel    = getdef(opts,'E_panel',   70e9);
    E_joint    = getdef(opts,'E_joint',   1e8);
    t_panel    = getdef(opts,'t_panel',   3e-3);
    nu_panel   = getdef(opts,'nu_panel',  0.33);
    m_node     = getdef(opts,'m_node',    10.0);
    joint_type = getdef(opts,'joint_type','triangle');
    panel_shear_factor = getdef(opts,'panel_shear_factor',1e-4);
    joint_shear_factor = getdef(opts,'joint_shear_factor',0.25);

    nRing  = 6;
    nHex   = 1 + nRing;
    d_ring = 3 * R;
    use_hub = strcmp(joint_type,'Y');

    %% --- Geometry (identical to Build_Hex_Ring_Model) ----------------------
    n_panel_raw = nHex * 6;
    max_raw     = n_panel_raw + 12 + 5;
    raw_coords  = zeros(max_raw, 3);
    raw_hex     = zeros(nHex, 6);

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
    n_placed = n_panel_raw;

    cr_hub_raw = zeros(nRing,1);  rr_hub_raw = zeros(nRing,1);
    if use_hub
        for k = 1:nRing
            theta_k = (k-1)*pi/3;
            n_placed = n_placed+1;  cr_hub_raw(k) = n_placed;
            raw_coords(n_placed,:) = [1.5*R*cos(theta_k), 0, 1.5*R*sin(theta_k)];
        end
        for k = 1:nRing
            k2 = mod(k,nRing)+1;
            p1 = raw_coords(raw_hex(k+1,6),:);
            p2 = raw_coords(raw_hex(k2+1,2),:);
            n_placed = n_placed+1;  rr_hub_raw(k) = n_placed;
            raw_coords(n_placed,:) = (p1+p2)/2;
        end
    end
    n_raw = n_placed;

    tol = R*1e-6;
    [node_coords, ~, ic] = uniquetol(raw_coords(1:n_raw,:), tol, ...
                                     'ByRows',true,'DataScale',1);
    hex_nodes = zeros(nHex,6);
    for h = 1:nHex
        for j = 1:6, hex_nodes(h,j) = ic(raw_hex(h,j)); end
    end
    center_nodes     = hex_nodes(1,:);
    inner_ring_nodes = hex_nodes(2:end,1);
    if use_hub, cr_hub = ic(cr_hub_raw);  rr_hub = ic(rr_hub_raw);
    else,       cr_hub = [];  rr_hub = [];  end
    nNodes = size(node_coords,1);

    %% --- Bar connectivity (identical to Build_Hex_Ring_Model) --------------
    max_bars = nHex*9 + 50;
    raw_bars = zeros(max_bars,2);
    is_joint_flag = false(max_bars,1);
    nb = 0;
    for h = 1:nHex
        v = hex_nodes(h,:);
        for j = 1:6, nb=nb+1; raw_bars(nb,:)=[v(j), v(mod(j,6)+1)]; end
        for j = 2:4, nb=nb+1; raw_bars(nb,:)=[v(1), v(j+1)]; end
    end
    if use_hub
        for k = 1:nRing
            nb=nb+1; raw_bars(nb,:)=[center_nodes(k), cr_hub(k)];      is_joint_flag(nb)=true;
            nb=nb+1; raw_bars(nb,:)=[cr_hub(k), inner_ring_nodes(k)];  is_joint_flag(nb)=true;
        end
        for k = 1:nRing
            k2 = mod(k,nRing)+1;
            nb=nb+1; raw_bars(nb,:)=[hex_nodes(k+1,6), rr_hub(k)];     is_joint_flag(nb)=true;
            nb=nb+1; raw_bars(nb,:)=[rr_hub(k), hex_nodes(k2+1,2)];    is_joint_flag(nb)=true;
        end
    else
        for k = 1:nRing
            nb=nb+1; raw_bars(nb,:)=[center_nodes(k), inner_ring_nodes(k)]; is_joint_flag(nb)=true;
        end
        for k = 1:nRing
            k2 = mod(k,nRing)+1;
            nb=nb+1; raw_bars(nb,:)=[hex_nodes(k+1,6), hex_nodes(k2+1,2)]; is_joint_flag(nb)=true;
        end
    end
    raw_bars_s = sort(raw_bars(1:nb,:),2);
    flag_s     = is_joint_flag(1:nb);
    [bar_conn, ~, ic_bar] = unique(raw_bars_s,'rows');
    nBars  = size(bar_conn,1);
    is_joint = false(nBars,1);
    for b = 1:nb
        if flag_s(b), is_joint(ic_bar(b)) = true; end
    end
    joint_ids = find(is_joint);

    %% --- 4-node rotational springs for panel bending -----------------------
    % Each hexagonal panel is triangulated by 3 interior diagonals from
    % vertex 1 to vertices 3, 4, 5. This divides the hexagon into 4
    % triangles sharing the 3 diagonals. For each interior edge (diagonal
    % shared between two triangles) we place a 4-node rotational spring.
    %
    % Triangle decomposition of hex vertices [v1..v6]:
    %   T1: v1-v2-v3  (edge v2-v3 is perimeter, v1-v3 is diagonal)
    %   T2: v1-v3-v4  (edge v3-v4 is perimeter, v1-v3 and v1-v4 are diag)
    %   T3: v1-v4-v5  (edge v4-v5 is perimeter, v1-v4 and v1-v5 are diag)
    %   T4: v1-v5-v6  (edge v5-v6 is perimeter, v1-v5 is diagonal)
    %
    % Interior shared edges and their 4N spring nodes (edge nodes i,j +
    % opposite apex of each adjacent triangle):
    %   edge v1-v3: triangles T1(apex=v2) and T2(apex=v4) → ijkl = [v1,v3,v2,v4]
    %   edge v1-v4: triangles T2(apex=v3) and T3(apex=v5) → ijkl = [v1,v4,v3,v5]
    %   edge v1-v5: triangles T3(apex=v4) and T4(apex=v6) → ijkl = [v1,v5,v4,v6]

    % Plate bending rigidity
    D_panel = E_panel * t_panel^3 / (12*(1-nu_panel^2));

    sprIJKL_raw = zeros(nHex*3, 4);
    sprK_raw    = zeros(nHex*3, 1);
    ns = 0;
    for h = 1:nHex
        v = hex_nodes(h,:);  % v(1)..v(6)
        % Interior edges: [v1-v3, v1-v4, v1-v5]
        interior_edges = [v(1),v(3),v(2),v(4);   % i,j, apex_left, apex_right
                          v(1),v(4),v(3),v(5);
                          v(1),v(5),v(4),v(6)];
        for e = 1:3
            ns = ns + 1;
            sprIJKL_raw(ns,:) = interior_edges(e,:);
            % Spring stiffness = D * edge_length (thin-plate crease model)
            edgeLen = norm(node_coords(interior_edges(e,1),:) - ...
                           node_coords(interior_edges(e,2),:));
            sprK_raw(ns) = D_panel * edgeLen;
        end
    end
    rot_spr_ijkl = sprIJKL_raw(1:ns,:);
    rot_spr_K    = sprK_raw(1:ns);

    %% --- Elements & assembly -----------------------------------------------
    node = Elements_Nodes;
    node.coordinates_mat = node_coords;
    node.mass_vec        = m_node * ones(nNodes,1);

    bar = Vec_Elements_Bars;
    bar.node_ij_mat = bar_conn;
    bar.A_vec       = barA * ones(nBars,1);
    E_vec           = E_panel * ones(nBars,1);
    E_vec(is_joint) = E_joint;
    bar.E_vec       = E_vec;

    rot_spr = Vec_Elements_RotSprings_4N;
    rot_spr.theta1 = 0;          % flat panels start at theta=0; disable small-angle
    rot_spr.theta2 = 1.9*pi;     % keep large-angle self-penetration guard
    rot_spr.node_ijkl_mat       = rot_spr_ijkl;
    rot_spr.rot_spr_K_vec       = rot_spr_K;

    assembly = Assembly_SolarPanel_Hex;
    assembly.node      = node;
    assembly.bar       = bar;
    assembly.rot_spr_4N = rot_spr;
    assembly.Initialize_Assembly();
    transverse_factor = panel_shear_factor * ones(nBars,1);
    transverse_factor(is_joint) = joint_shear_factor;
    assembly.transverse_bar_ids = (1:nBars)';
    assembly.transverse_K_vec   = transverse_factor .* ...
        (bar.E_vec .* bar.A_vec ./ bar.L0_vec);

    %% --- Convenience node sets --------------------------------------------
    outer_ring1 = setdiff(hex_nodes(2,:), inner_ring_nodes(1));
    ring4_nodes = unique(hex_nodes(5,:));

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
    model.panel_shear_factor = panel_shear_factor;
    model.joint_shear_factor = joint_shear_factor;
    model.transverse_bar_ids = assembly.transverse_bar_ids;
    model.transverse_K     = assembly.transverse_K_vec;
    model.R                = R;
    model.nHex             = nHex;
    model.nRing            = nRing;
    % New fields vs. Build_Hex_Ring_Model:
    model.rot_spr_ijkl     = rot_spr_ijkl;
    model.rot_spr_K        = rot_spr_K;
    model.t_panel          = t_panel;
    model.nu_panel         = nu_panel;
    model.D_panel          = D_panel;

    fprintf('Build_SolarPanel_Hex_Model: %d nodes, %d bars (%d joints), %d bending springs\n', ...
            nNodes, nBars, numel(joint_ids), ns);
end

function v = getdef(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end
