% cvx_begin quiet
%     variable Ex_x(6,samples)
%     variable U(2,1)
%     variable lam(2,samples) binary;
%     variable lam_temp(samples,1) binary;
%     minimize (U'*U_quad_cost*U + U_lin_cost*U)
%     subject to
%         for n = 1:samples
%             Ex_x(:,n) == Samples_A(:,:,n) * X_0 + B * U;
%             T_bound_A * Ex_x(:,n) - T_bound_b <= 1000 .* lam(:,n);
%             sum(lam(:,n)) <= 1000 * lam_temp(n)
%         end
%         U_bound_A * U <= U_bound_b;
%         sum(lam_temp)/samples <= alpha;
% cvx_end


cvx_begin quiet
    variable Ex_x(6,samples)
    variable U(2,1)
    minimize (U'*U_quad_cost*U + U_lin_cost*U)
    subject to
        for n = 1:samples
            Ex_x(:,n) == Samples_A(:,:,n) * X_0 + B * U;
            T_bound_A * Ex_x(:,n) <=  T_bound_b;
        end
        U_bound_A * U <= U_bound_b;
cvx_end
