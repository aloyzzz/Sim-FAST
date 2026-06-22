%% Bar elements 
% This bar element is calculated based on analytical equations
% This code is vectorized for speed
% This formulation gives a linear elastic response
% The bar geometry is defined with two nodes

classdef Vec_Elements_Bars < handle

    properties
        % Connection information of the bar, stored as a matrix (Nb*2)
        node_ij_mat

        % Area of the bar, stored as a vector (Nb*1)
        A_vec

        % Young's Modulus of the bar, stored as a vector (Nb*1)
        E_vec

        % Length of the bar, stored as a vector (Nb*1)
        L0_vec

        % Current Engineering Strain of the bar, stored as a vector (Nb*1)
        strain_current_vec

        % Current Strain Energy of the bar, stored as a vector (Nb*1)
        energy_current_vec

        % Eigenstrain (prestrain) offset per bar (Nb*1), default zeros.
        % Sx = E*(Ex - prestrain_vec). Negative value = bar wants to shorten
        % (contraction actuator). Updated each timestep by the solver for
        % actuated bars; zero for passive bars.
        prestrain_vec

    end

    methods
        % Initialize the original length of bar
        Initialize(obj,node)

        % Calculate the strain of bars
        [Ex]=Solve_Strain(obj,node,U);

        % Calculate the stress and stiffness of bars
        % This function defines the constitutive model for bars
        [Sx,Cx]=Solve_Stress(obj,Ex)

        % Calculate the global force vector
        [Tbar]=Solve_Global_Force(obj,node,U,Sx)

        % Calculate the global stiffness matrix
        [Kbar]=Solve_Global_Stiff(obj,node,U,Sx,C)
        
        % This function is the main function we use to interact with the
        % solver. We use this function to compute the global forces and 
        % stiffness of the bar elements (making use of the above four 
        % functions). 
        [Tbar,Kbar]=Solve_FK(obj,node,U)

    end
end
