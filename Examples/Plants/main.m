% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clean env
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear;
clc;
cvx_clear;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system setup
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_0 = [.958; 1.063; 1.196];

B = [zeros(1,2); -0.0025, 0; -0.0015, -0.003];

U_G = [1, 1; -1, 0; 0, -1];
U_h = [1000; 0; 0];

G   =[ 0, 0, 1];
h_1 = 0.9;
h_2 = 0.8;

alpha = 3/20;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% proposed method
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
exp_calc = @(i) (i^4)/81;
var_calc = @(i) (i^4)/(81^2);

Ex_A = [zeros(2,1), eye(2); exp_calc(1), exp_calc(2), exp_calc(3)];

exp_1_state = exp_calc(1)*x_0(1) + exp_calc(2)*x_0(2) + exp_calc(3)*x_0(3);
var_1_state = [var_calc(1), var_calc(2), var_calc(3)] * (x_0.^2);

var_2_state = x_0(2)^2*var_calc(1) + ...
              x_0(3)^2*var_calc(2) + ...
              (var_calc(3)+exp_calc(3)^2) * var_1_state + ...
              var_calc(3) * exp_1_state^2;
var_2_u1 = (x_0(3)*B(2,1)*var_calc(2) + B(3,1)*exp_1_state*var_calc(3))/2;
var_2_u2 = (B(3,2)*exp_1_state*var_calc(3))/2;
var_2_u1u2 = (B(3,1)*B(3,2)*var_calc(3))/2;
var_2_u1_2 = B(2,1)^2*var_calc(2)+B(3,1)^2*var_calc(3);
var_2_u2_2 = B(3,2)^2*var_calc(3);

var_2_mat =[var_2_state, var_2_u1,    var_2_u2;
            var_2_u1,    var_2_u1_2,  var_2_u1u2;
            var_2_u2,    var_2_u1u2,  var_2_u2_2];
var_2_norm = chol(var_2_mat);
             
lam = sqrt(2/alpha) * ones(2,1);
u_p = zeros(4,1);

cvx_solver mosek
tic
for i = 1:50
    
    solve_u    
    opt_u = cvx_optval;
    
    solve_lam
    opt_l = cvx_optval;
    
    if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
        return
    end
    
    comp = norm(u_p - [u_0;u_1]);
    u_p = [u_0;u_1];
    fprintf('iteration: %d ', i);
    fprintf('\t %f', opt_u);
    fprintf('\t %f \n', opt_l);
    
    if  comp <= 1e-4
        break
    end
end
proposed_time = toc


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve with particle control
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%

solve_pc

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

verify


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make plots
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
colors = [224,   0,   0; % red
           30, 144  255; % dark blue
            0, 170,  85; % green
          118,   0, 168; % purple
           46,  52,  59; % grey
          236, 176,  31;  % yellow
           76, 189, 237; % light blue
          161,  19,  46  % dark red
           ] ./ 255;
       
ex1 = Ex_A*x_0;
ex2 = Ex_A*Ex_A*x_0;
hold on
p1=plot([0:2],[x_0(3); Ex_x_1(3); Ex_x_2(3)], 'o-', "Color", colors(4,:));
p2=plot([0:2],[x_0(3); mean(x_1(3,:)); mean(x_2(3,:))], 'd-', "Color", colors(3,:));

p3=plot([-2:2],[x_0; ex1(3); ex2(3)], '-', "Color", colors(2,:));
p6=plot([0:2],[x_0(3);0.9;0.8], 's-', "Color", colors(1,:));

ylim([0 2])
xlabel("Time Step, k")
xticks([-2:2])
ylabel("$\bf{E}[\rho(k)]$", "interpreter", "latex")
legend([p1,p3,p2,p6],...
    {'Proposed Method', 'No Input', 'Particle Control',  'Target'}, ...
    "interpreter", "latex", ...
    'Orientation','horizontal', ...
    'Location', 'northwest', ...
    'NumColumns', 2)
hold off
