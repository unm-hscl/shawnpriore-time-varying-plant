cvx_begin quiet
    variable Ex_x(6,1)
    variable U(2,1)
    variable lam(2,1);
    variable lam_temp(2,1);
    minimize (U'*U_quad_cost*U + U_lin_cost*U)
    subject to
        Ex_x == Ex_A * X_0 + B * U;
        T_bound_A * Ex_x + lam .* sqrt(Var_T) <= T_bound_b;
        U_bound_A * U <= U_bound_b;
        lam >= sqrt(5/3);
        for i = 1:2
            lam_temp(i) >= pow_func_m * lam(i) + pow_func_c;
        end
        sum(lam_temp) <= alpha;
cvx_end
