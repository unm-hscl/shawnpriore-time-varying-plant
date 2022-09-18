x_0 = [.958; 1.063; 1.196];
B = [zeros(1,2); -0.0025, 0; -0.0015, -0.003];

N_samples = 50000;

x_1 = zeros(N_samples, 1);
x_2 = zeros(N_samples, 1);

for i = 1:N_samples
    A1 = [zeros(2,1), eye(2); gamrnd(1,1/81), gamrnd(16,1/81), gamrnd(81,1/81)];
    A2 = [zeros(2,1), eye(2); gamrnd(1,1/81), gamrnd(16,1/81), gamrnd(81,1/81)];
    x1 = A1*x_0+B*[100;100];
    x2 = A2*x1+B*[100;100];
    
    x_1(i) = max(0,x1(3));
    x_2(i) = max(0,x2(3));
end

%%
figure()
subplot(1,2,1);
histogram(x_1);
subplot(1,2,2);
histogram(x_2);
