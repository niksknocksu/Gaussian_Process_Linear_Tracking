% author : nikhil.sharma.mcmaster@gmail.com

function MAIN_GAUSSIAN_PROCESS_TRACKING()

addpath './tracking models related'

% Check xml files in the following folder for target and sensor parameters
addpath './tracking models related/config_target'
addpath './tracking models related/config_sensor' 


target_1 = constant_acc('config_target/config_ca.xml');
target_1.initialize();
target_1.gen_truth();

linear_sensor_1 = linear_sensor('config_sensor/config_linear_sensor.xml');
linear_sensor_1.initialize()

num_samples = size(target_1.x_truth, 2);
z = zeros(linear_sensor_1.dim, num_samples);
for i = 1:num_samples
    z(:, i) = linear_sensor_1.get_meas(target_1.x_truth(1,i), target_1.x_truth(2,i));
end

sliding_window = 4; % use last N measurements to optimize Gaussian process
x_estimate_KF = zeros(4, size(target_1.x_truth,2));
x_estimate_GP = zeros(4, size(target_1.x_truth,2));

x_estimate_KF(1:4,1) = [z(:,1); 0 ; 0]; % initialize velocity to 0
R = linear_sensor_1.get_R();
H = [1,0,0,0;...
    0,1,0,0];
P_KF = blkdiag(R, 100, 100);


% Deliberate target mismatch, constant vel KF used for a CA target
F = [1, 0, 1, 0;...
    0, 1, 0, 1;...
    0, 0, 1, 0;...
    0, 0, 0, 1];
T = 1; %sampling time in seconds
Q = target_1.q_tilde .* [T^3/3 0 T^2/2 0;...
   0 T^3/3 0 T^2/2;...
   T^2/2 0 T 0;...
   0 T^2/2 0 T]; 

% GP recursion for 1st estimate
sw = 1;
modelOut_train = z(:, 1);
x_train = modelOut_train(1,:)';
y_train = modelOut_train(2,:)';
[ln_scale_x, sigma_f_x, sigma_n_x] = train_gp(x_train, sw, sqrt(R(1,1)));
[ln_scale_y, sigma_f_y, sigma_n_y] = train_gp(y_train, sw, sqrt(R(2,2)));
K = se_kernel(sw, sw, ln_scale_x, sigma_f_x) + sigma_n_x^2 * eye(numel(x_train));
K_s = se_kernel(sw, 1, ln_scale_x, sigma_f_x);
K_ss = se_kernel(1, 1, ln_scale_x, sigma_f_x) + 1e-8;

mu_x = K_s' / K * x_train;
cov_x = K_ss - K_s'/K * K_s;

% predict current point - y coord
K = se_kernel(sw, sw, ln_scale_y, sigma_f_y) + sigma_n_y^2 * eye(numel(y_train));
K_s = se_kernel(sw, 1, ln_scale_y, sigma_f_y);
K_ss = se_kernel(1, 1, ln_scale_y, sigma_f_y) + 1e-8;
mu_y =  K_s' / K * y_train;
cov_y = K_ss - K_s'/K * K_s;

x_estimate_GP(1:2,1) = [mu_x; mu_y];



for t = 2:num_samples
    fprintf('Starting time-step : %d /n', t)

    % KF recursion, predict and update
    x_estimate_KF(:,t) = F * x_estimate_KF(:, t-1);
    P_KF = F * P_KF * F' + Q;
    K =  P_KF * H' /(H * P_KF * H' + R);
    x_estimate_KF(:, t) = x_estimate_KF(:, t) + K * (z(:, t) - x_estimate_KF(1:2, t));
    P_KF = (eye(size(P_KF, 1)) - K*H) * P_KF;

    % Gaussian Process recursion
    sw = get_window(t, sliding_window, T)'; % T_train
    modelOut_train = z(:, sw);

    % train Gaussian process
    x_train = modelOut_train(1,:)';
    y_train = modelOut_train(2,:)';

    [ln_scale_x, sigma_f_x, sigma_n_x] = train_gp(x_train, sw, sqrt(R(1,1)));
    [ln_scale_y, sigma_f_y, sigma_n_y] = train_gp(y_train, sw, sqrt(R(2,2)));
    
    % R(1,1) = sigma_n_x;
    % R(2,2) = sigma_n_y;
    % predict current point - x coord
    K = se_kernel(sw, sw, ln_scale_x, sigma_f_x) + sigma_n_x^2 * eye(numel(x_train));
    K_s = se_kernel(sw, t, ln_scale_x, sigma_f_x);
    K_ss = se_kernel(t, t, ln_scale_x, sigma_f_x) + 1e-8;

    mu_x = K_s' / K * x_train;
    cov_x = K_ss - K_s'/K * K_s;
    
    % predict current point - y coord
    K = se_kernel(sw, sw, ln_scale_y, sigma_f_y) + sigma_n_y^2 * eye(numel(y_train));
    K_s = se_kernel(sw, t, ln_scale_y, sigma_f_y);
    K_ss = se_kernel(t, t, ln_scale_y, sigma_f_y) + 1e-8;

    mu_y =  K_s' / K * y_train;
    cov_y = K_ss - K_s'/K * K_s;
    
    x_estimate_GP(1:2,t) = [mu_x; mu_y]; 
end


figure(1)
hold on
plot(target_1.x_truth(1,:), target_1.x_truth(2,:), '*-')
plot(z(1,:), z(2,:), '+')
plot(x_estimate_KF(1, : ), x_estimate_KF(2, :), '^-');
plot(x_estimate_GP(1,:), x_estimate_GP(2,:), 'o-');
legend('True Object', 'Measurements', 'KF', 'GP', 'Location','best')
xlabel('X (meters)')
ylabel('Y (meters)')

% Remove the first point as GP estimate was far off
squared_error_x_GP = (x_estimate_GP(1,2:end) - target_1.x_truth(1,2:end)).^2;
squared_error_x_KF = (x_estimate_KF(1,2:end) - target_1.x_truth(1,2:end)).^2;
squared_error_y_GP = (x_estimate_GP(2,2:end) - target_1.x_truth(2,2:end)).^2;
squared_error_y_KF = (x_estimate_KF(2,2:end) - target_1.x_truth(2,2:end)).^2;


figure(2)
hold on
plot(squared_error_x_GP);
plot(squared_error_x_KF);
legend('Gaussian Process',' KF')
xlabel('Time Steps')
ylabel('Squared error')

figure(3)
hold on
plot(squared_error_y_GP);
plot(squared_error_y_KF);
legend('Gaussian Process',' KF')
xlabel('Time Steps')
ylabel('Squared Error')

end


function sw = get_window(current_time, window_size, sample_time)
    if current_time < window_size
        sw = 1:sample_time:current_time;
    elseif current_time >= window_size
        sw = (current_time - window_size) + 1 : sample_time : current_time;
    end

end


function [length_scale, sigma_f, sigma_n] = train_gp(y_train, t_train, sigma_n)
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
    cost = @(theta) negative_log_likelihood(t_train, y_train, theta);
    opt_theta = fmincon(cost, [sigma_n, sigma_n, sigma_n], [], [], [], [], [1e-5, 1e-5, 1e-5], [], [], options);
    
    length_scale = opt_theta(1);
    sigma_f = opt_theta(2);
    sigma_n = opt_theta(3);
end


function nll = negative_log_likelihood(in_train, out_train, theta)
    length_scale = theta(1);
    sigma_f = theta(2);
    sigma_n = theta(3);
    n = numel(out_train);
    K = se_kernel(in_train, in_train, length_scale, sigma_f);
    K = K + sigma_n^2 * eye(size(K,1));
    % L = chol(K, 'lower');
    % 
    % opts.LT = true;
    % S1 = linsolve(L, out_train, opts);
    % opts.LT = false;
    % S2 = linsolve(L', S1, opts);

    % nll = sum(log(diag(L))) + 0.5 * out_train' * S2 + 0.5 * numel(in_train) * log(2*pi);

    nll = 0.5 * ((out_train' / K) * out_train) + 0.5 * log(det(K)) +  0.5 * n * log(2*pi); % An Intuitive Tutorial to Gaussian Process Regression
end


