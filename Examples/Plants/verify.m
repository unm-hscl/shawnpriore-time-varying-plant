N = 10000;

pc_correct=zeros(1,N);
pm_correct=zeros(1,N);

% randomly generate the disturbance vector from the multivariate t.
for i = 1:N
    A1_rnd = [zeros(2,1), eye(2); gamrnd(1,1/81), gamrnd(16,1/81), gamrnd(81,1/81)];
    A2_rnd = [zeros(2,1), eye(2); gamrnd(1,1/81), gamrnd(16,1/81), gamrnd(81,1/81)];
    
    pm_x_1 = A1_rnd*x_0 + B*u_0;
    pm_x_2 = A2_rnd*pm_x_1 + B*u_1;
    pc_x_1 = A1_rnd*x_0 + B*U_1;
    pc_x_2 = A2_rnd*pc_x_1 + B*U_2;
    
    if (G*pm_x_1 <= h_1) && (G*pm_x_2 <= h_2)
        pm_correct(i)=1;
    end
    if (G*pc_x_1 <= h_1) && (G*pc_x_2 <= h_2)
        pc_correct(i)=1;
    end
end

pm_per = sum(pm_correct)/N
pc_per = sum(pc_correct)/N

