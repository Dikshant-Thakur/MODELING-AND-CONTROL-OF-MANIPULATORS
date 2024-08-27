function J = GetJacobian (biTei, bTe, jointType)
n_links = size(biTei, 3);
    J = ones(6,n_links);
    o_r_end = bTe(1:3,4);
    
    for i = 1:1:n_links
        T_i = GetTransformationWrtBase(biTei, i);
        k_i = T_i(1:3,3);

        if jointType(i) == 0 % rotational join 
            J(1:3,i) = k_i;
            o_r_i = T_i(1:3,4);
            i_r_end = o_r_end - o_r_i;
            J(4:6,i) = cross(k_i, i_r_end);
        
        elseif jointType(i) == 1 % prismatic joint
            J(1:3, i) = zeros(3,1);
            J(4:6,i) = k_i(1:3);
        end
    end