%% Assembly_SolarPanel_Hex
%
% Assembly class for hexagonal solar-panel arrays with bending stiffness.
%
% Extends the bar-only Assembly_Hex_Origami by adding 4-node rotational
% spring elements at the interior triangulation edges of each panel. These
% springs represent the out-of-plane bending stiffness (flexural rigidity D)
% of a thin solar-panel plate, preventing zero-energy floppy modes in full
% 3-D simulations where the out-of-plane (Y) direction is left unconstrained.
%
% Usage:
%   assembly = Assembly_SolarPanel_Hex;
%   assembly.node     = <Elements_Nodes object>;
%   assembly.bar      = <Vec_Elements_Bars object>;
%   assembly.rot_spr_4N = <Vec_Elements_RotSprings_4N object>;
%   assembly.Initialize_Assembly();
%   [T, K] = assembly.Solve_FK(U);

classdef Assembly_SolarPanel_Hex < handle

    properties
        node          % Elements_Nodes
        bar           % Vec_Elements_Bars  (panel bars + joint bars)
        rot_spr_4N    % Vec_Elements_RotSprings_4N (panel bending springs)
        transverse_bar_ids = [] % bars with transverse Y stiffness
        transverse_K_vec = []   % N/m, one value per transverse_bar_ids
    end

    methods
        [T, K] = Solve_FK(obj, U)
        Initialize_Assembly(obj)
    end

end
