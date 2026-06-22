function [T,K] = Solve_FK(obj,U)
    [T,K] = obj.bar.Solve_FK(obj.node,U);
end
