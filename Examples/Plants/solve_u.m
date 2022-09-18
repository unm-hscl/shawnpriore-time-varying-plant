cvx_begin quiet
    variable Ex_x_1(3,1) nonnegative
    variable u_0(2,1) nonnegative
    
    variable Ex_x_2(3,1) nonnegative
    variable u_1(2,1) nonnegative
           
    minimize (u_0'*u_0 + u_1'*u_1)
    subject to
        Ex_x_1 == Ex_A * x_0 + B * u_0;
        Ex_x_2 == Ex_A * Ex_x_1 + B * u_1;
        
        U_G * u_0 <= U_h;
        U_G * u_1 <= U_h;
        
        G*Ex_x_1 + lam(1)*sqrt(var_1_state) <= h_1;
        
        G*Ex_x_2 + lam(2)*norm(var_2_norm * [1; u_0]) <= h_2;
                
cvx_end