function Initialize_Assembly(obj)

    obj.node.current_U_mat         = zeros(size(obj.node.coordinates_mat));
    obj.node.current_ext_force_mat = zeros(size(obj.node.coordinates_mat));

    obj.bar.Initialize(obj.node);

    % theta_stress_free_vec is set here to whatever the CURRENT geometry
    % is -- i.e. your as-assembled panel angles become the zero-torque
    % reference state for the rotational springs.
    obj.rot_spr_3N.Initialize(obj.node);

end
