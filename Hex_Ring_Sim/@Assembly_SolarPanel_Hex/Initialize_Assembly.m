function Initialize_Assembly(obj)
%% Initialize_Assembly  (Assembly_SolarPanel_Hex)
% Resets displacements to zero and initialises bar and rotational-spring
% elements from the current reference geometry.

    obj.node.current_U_mat         = zeros(size(obj.node.coordinates_mat));
    obj.node.current_ext_force_mat = zeros(size(obj.node.coordinates_mat));

    obj.bar.Initialize(obj.node);
    obj.rot_spr_4N.Initialize(obj.node);

end
