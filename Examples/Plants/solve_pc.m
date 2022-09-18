%%%%%%%%%%%%%%%%%%%%%%%%
% Blackmore 2011 for goal achievement and collision avoidance
% Primary Coder: Vignesh Sivaramakrishnan
% Modified by: Shawn Priore
%%%%%%%%%%%%%%%%%%%%%%%%

% parameters
% big M arbitrary constant
large_constant = 5000;
N = 250;

% randomly generate the disturbance vector from the multivariate t.
A1_rnd = zeros(3,3,large_constant);
A2_rnd = zeros(3,3,large_constant);
for i = 1:large_constant
    A1_rnd(:,:,i) = [zeros(2,1), eye(2); gamrnd(1,1/81), gamrnd(16,1/81), gamrnd(81,1/81)];
    A2_rnd(:,:,i) = [zeros(2,1), eye(2); gamrnd(1,1/81), gamrnd(16,1/81), gamrnd(81,1/81)];
end

%% Run optimization problem for an optimal control policy
% We run an optimization problem to determine the control policy over the
% time horizon T.

tic;
cvx_begin quiet
    variable U_1(2);
    variable U_2(2);
    variable x_1(3,N);
    variable x_2(3,N);
    variable t_1(N) binary;
    variable t_2(N) binary;
    variable t(N) binary;

    minimize (U_1'*U_1 + U_2'*U_2);

    subject to
    
    
        U_G * U_1 <= U_h;
        U_G * U_2 <= U_h;
        
        for i = 1:N
            x_1(:,i) == A1_rnd(:,:,i)*x_0 + B*U_1;
            x_2(:,i) == A2_rnd(:,:,i)*x_1(:,i) + B*U_2;
            G*x_1(:,i) - h_1 <= t_1(i)*large_constant;
            G*x_2(:,i) - h_2 <= t_2(i)*large_constant;
            t_1(i) + t_2(i) <= t(i)*large_constant;
        end
        1/N * sum(t) <= alpha;
                       
cvx_end
total_time_pc = toc;

% print some useful information
fprintf('%s \n', cvx_status);
fprintf('Computation time (sec): %f \n', total_time_pc);
fprintf('Input Cost: %f \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end