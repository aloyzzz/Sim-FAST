function  [Sx,Cx]= Solve_Stress(obj,Ex)

    Sx=obj.E_vec.*(Ex - obj.prestrain_vec);
    Cx=obj.E_vec;

end