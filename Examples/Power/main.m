% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clean env
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear;
clc;
cvx_clear;
cvx_solver mosek

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system setup
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%

C_wind = 100 * 0.5 * 1.225 * pi * 65^2 / 1e6;
L_max = 1600;
X_0 = [0; 0; C_wind; 0; L_max; 0];


B = [1, 0;
     0, 1;
     0, 0;
     0, 0;
     0, 0;
     0, 0];
    
U_bound_A = [-eye(2); eye(2)];
U_bound_b = [60; 60; 600; 600];

T_bound_A = [-1, -1, 0, -1, 0, 1;
              1,  0, 0,  1, 0, 0];
T_bound_b = [0; 900];

U_quad_cost = [0.05, 0; 0, 0.1];
U_lin_cost = [30, 60];


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% proposed method setup
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
w_lambda = 5;
w_kappa = 30;
b_alpha = 50;
b_beta = 50;

Ex_weibul3 = @(lambda, kappa) lambda^3*gamma(1+3/kappa);
Var_weibul3 = @(lambda, kappa) lambda^6*(gamma(1+6/kappa)-gamma(1+3/kappa)^2);
Ex_beta = @(alpha, beta) alpha/(alpha+beta); 
Var_beta = @(alpha, beta) alpha*beta / ((alpha+beta)^2*(alpha+beta+1));

Ex_A = [0, 0, 0,                            0, 0,                       0;
        0, 0, 0,                            0, 0,                       0;
        0, 0, 1,                            0, 0,                       0;
        0, 0, Ex_weibul3(w_lambda,w_kappa), 0, 0,                       0;
        0, 0, 0,                            0, 1,                       0;
        0, 0, 0,                            0, Ex_beta(b_alpha,b_beta), 0];


Var_T = [Var_weibul3(w_lambda,w_kappa) * C_wind^2 + Var_beta(b_alpha,b_beta) * L_max^2; Var_weibul3(w_lambda,w_kappa) * C_wind^2];

pow_func = @(x) 4./(9*(x.^2+1));
[pow_func_m, pow_func_c] = function_affine(1e-4, 30, sqrt(5/3), pow_func, 1e-4, sqrt(5/3));

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% proposed method timing
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
our_cost = zeros(16,1);
our_time = zeros(16,1);
for j = 1:16
    disp(j);
    alpha = 0.17 - 0.01 * j;
    tic;
    solve_our_method 
    if strcmpi(cvx_status, 'Infeasible') || strcmpi(cvx_status, 'Failed')
        break
    end
    our_time(j) = toc;
    our_cost(j) = cvx_optval;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% scp set up and timing
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
confidence = 0.001;
scp_cost = zeros(16,1);
scp_time = zeros(16,1);
for j = 1:16
    disp(j);
    alpha = 0.17 - 0.01 * j;

    samples = ceil((2/alpha)*(log(1/confidence)+2));
    Samples_A = zeros(6, 6, samples);
    for k = 1:samples
        Samples_A(3,3,k) = 1;
        Samples_A(4,3,k) = wblrnd(w_lambda,w_kappa)^3;
        Samples_A(5,5,k) = 1;
        Samples_A(6,5,k) = betarnd(b_alpha,b_beta);
    end

    tic;
    solve_scp 
    if strcmpi(cvx_status, 'Infeasible') || strcmpi(cvx_status, 'Failed')
        break 
    end
    scp_time(j) = toc;
    scp_cost(j) = cvx_optval;
end

%%

alphas = [0.84:.01:0.99];
figure()
hold on
plot(alphas(1:end-1), our_cost(1:end-1), '-o', 'DisplayName','Proposed Method')
plot(alphas, scp_cost, '-s', 'DisplayName','Scenario Approach')
xlabel("Safety probability, $1-\alpha$", 'interpreter', 'latex')
ylabel("Cost, in dollars", 'interpreter', 'latex')
legend('location', 'northwest', 'interpreter', 'latex')
hold off
figure()
hold on
plot(alphas(1:end-1), our_time(1:end-1), '-o', 'DisplayName','Proposed Method')
plot(alphas, scp_time, '-s', 'DisplayName','Scenario Approach')
xlabel("Safety probability, $1-\alpha$", 'interpreter', 'latex')
ylabel("Time to solve, in seconds", 'interpreter', 'latex')
legend('location', 'northwest','interpreter', 'latex')
hold off