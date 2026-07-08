function [T, K] = Solve_FK(obj, U)
%% Solve_FK  (Assembly_SolarPanel_Hex)
% Returns the global internal force vector T and tangent stiffness K by
% summing contributions from bar elements and 4-node rotational springs.

    [Tbar, Kbar]   = obj.bar.Solve_FK(obj.node, U);
    [Tspr, Kspr]   = obj.rot_spr_4N.Solve_FK(obj.node, U);

    T = Tbar + Tspr;
    K = Kbar + Kspr;

    if ~isempty(obj.transverse_bar_ids)
        [Tsh, Ksh] = Solve_Transverse_Bar_Stiffness(obj, U);
        T = T + Tsh;
        K = K + Ksh;
    end

end

function [Tsh, Ksh] = Solve_Transverse_Bar_Stiffness(obj, U)
%% Solve_Transverse_Bar_Stiffness
% Adds linear transverse stiffness between bar endpoint Y displacements.
% Axial bar elements have zero first-order Y stiffness when flat because
% every bar lies in the XZ plane. These springs are a compact plate/joint
% surrogate so panel bending and joint shear are present before large
% geometric cable stiffening appears.

    nodeNum = size(obj.node.coordinates_mat, 1);
    Tsh = zeros(3*nodeNum, 1);
    Ksh = sparse(3*nodeNum, 3*nodeNum);

    barIds = obj.transverse_bar_ids(:);
    kVec   = obj.transverse_K_vec(:);
    conn   = obj.bar.node_ij_mat;

    for q = 1:numel(barIds)
        n1 = conn(barIds(q), 1);
        n2 = conn(barIds(q), 2);
        d1 = 3*(n1-1) + 2;
        d2 = 3*(n2-1) + 2;
        k  = kVec(q);
        du = U(n1,2) - U(n2,2);

        Tsh(d1) = Tsh(d1) + k*du;
        Tsh(d2) = Tsh(d2) - k*du;

        Ksh(d1,d1) = Ksh(d1,d1) + k;
        Ksh(d1,d2) = Ksh(d1,d2) - k;
        Ksh(d2,d1) = Ksh(d2,d1) - k;
        Ksh(d2,d2) = Ksh(d2,d2) + k;
    end
end
