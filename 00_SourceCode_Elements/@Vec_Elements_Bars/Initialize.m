function Initialize(obj,node)

    obj.L0_vec      = zeros(size(obj.A_vec));
    obj.prestrain_vec = zeros(size(obj.A_vec));
    % The primary task for initializing code is to calculate the 
    % undeformed length of each bar element. This is done with the 
    % following simple for loop.

    for i=1:length(obj.A_vec)
        node1=node.coordinates_mat(obj.node_ij_mat(i,1),:);
        node2=node.coordinates_mat(obj.node_ij_mat(i,2),:);
        obj.L0_vec(i)=norm(node1-node2);        
    end

end
