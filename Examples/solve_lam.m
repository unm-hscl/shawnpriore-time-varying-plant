pow_func = @(x) 4./(9*(x.^2+1));
[pow_func_m, pow_func_c] = function_affine(1e-3, 20, sqrt(5/3), pow_func, 1e-3, sqrt(5/3));

cvx_begin  quiet
    variable lam(2)
    variable lam_temp(2)
    
    minimize (sum(lam))
    subject to
                
        G*Ex_x_1 + lam(1)*sqrt(var_1_state) <= h_1;
        G*Ex_x_2 + lam(2)*norm(var_2_norm * [1; u_0]) <= h_2;
        
        for j = 1:2
            lam_temp(j) >= pow_func_m .* lam(j) + pow_func_c;
        end
        sum(lam_temp) <= alpha;
        lam >= sqrt(5/3);
cvx_end