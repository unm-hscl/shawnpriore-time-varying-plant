cvx_begin  quiet
    variable lam(2)
    
    minimize (sum(lam))
    subject to
                
        G*Ex_x_1 + lam(1)*sqrt(var_1_state) <= h_1;
        G*Ex_x_2 + lam(2)*norm(var_2_norm * [1; u_0]) <= h_2;
        
        sum(pow_p(lam,-2)) <= 9/4*alpha;
        lam >= sqrt(8/3);
cvx_end