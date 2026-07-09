%% Assembly: hexagon panels connected by bars + 3-node rotational springs
%
% Identical to Assembly_Hex_Origami, but adds a rot_spr_3N property so
% joints can carry angular (bending) stiffness in addition to the axial
% stiffness of the bar elements. Use this whenever your joints include
% a hub node (i.e. joint_type = 'Y' in the hex-ring scripts) that can
% serve as the vertex of the angle spring.

classdef Assembly_Hex_Origami_RotSpr < handle
    properties
        node
        bar
        rot_spr_3N
    end
    methods
        [T,K] = Solve_FK(obj,U)
        Initialize_Assembly(obj)
    end
end
