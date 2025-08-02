clc; clear; close all;

% Population and Iteration Settings
nPop = 20;        % Number of individuals
maxIter = 100;    % Maximum number of iterations

% Parameter Bounds [Kp, Ki, Kd, lambda, mu]
lb = [3, 0.1, 3, 0, 0];    % Lower bounds: Kp, Ki, Kd, lambda, mu
ub = [5, 0.25, 5, 2, 2];     % Upper bounds: Kp, Ki, Kd, lambda, mu

% Run Optimization using M-AHA
optimal_params = M_AHA(@objective_function, nPop, maxIter, lb, ub);

% Display Optimized FOPID Parameters
disp('Optimized FOPID Parameters:');
disp(['Kp: ', num2str(optimal_params(1))]);
disp(['Ki: ', num2str(optimal_params(2))]);
disp(['Kd: ', num2str(optimal_params(3))]);
disp(['Lambda: ', num2str(optimal_params(4))]);
disp(['Mu: ', num2str(optimal_params(5))]);

%% Plotting the Optimized Controller Performance
% Given Constants (same as in the objective function)
Ca = 1.19; 
M = 1500; 
tau = 0.2; 
T = 1; 
C1 = 743; 
V0 = 20;  % Example reference speed

% Compute Transfer Function Parameters
C = C1 / (M * T * tau);
p1 = -2 * (Ca * V0 / M);
p2 = -1 / T;
p3 = -1 / tau;

% Define Transfer Function G(s)
s = tf('s');
G = C / ((s - p1) * (s - p2) * (s - p3));

% Define the Optimized FOPID Controller using the obtained parameters
Kp_opt = optimal_params(1);
Ki_opt = optimal_params(2);
Kd_opt = optimal_params(3);
lambda_opt = optimal_params(4); % Not used in classical pid() below
mu_opt = optimal_params(5);     % Not used in classical pid() below

% For simplicity, we use the classical PID representation.
% For a true fractional-order controller, you may use fotf() from the FOMCON toolbox.
C_fopid_opt = pid(Kp_opt, Ki_opt, Kd_opt);

% Create the closed-loop system with the optimized controller
T_cl_opt = feedback(C_fopid_opt * G, 1);

% Plot Step Response
figure;
step(T_cl_opt);
title('Step Response of Optimized FOPID Controlled System');
grid on;

% Plot Bode Plot
figure;
bode(C_fopid_opt * G);
title('Bode Plot of Optimized FOPID Controlled System');
grid on;
