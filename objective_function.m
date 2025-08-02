function F = objective_function(x)
    % OBJECTIVE_FUNCTION Evaluates performance for FOPID cruise control optimization.
    %
    % Input:
    %   x : A vector of FOPID parameters [Kp, Ki, Kd, lambda, mu]
    %
    % Output:
    %   F : A scalar cost value to be minimized.

    % Extract FOPID parameters
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);
    lambda = x(4);
    mu = x(5);

    % Given Constants
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

    % Define FOPID Controller
    % (For a true fractional-order controller, you may use fotf() from FOMCON Toolbox)
    C_fopid = pid(Kp, Ki, Kd);
    % C_fopid = fotf(Kp, Ki, lambda, Kd, mu); % Uncomment if using FOMCON Toolbox

    % Closed-loop system
    T_cl = feedback(C_fopid * G, 1);

    % Simulate Step Response
    [y, t] = step(T_cl);
    
    % Extract Performance Metrics
    OS = (max(y) - 1) * 100;  % Percent Overshoot
    Ess = abs(1 - y(end));     % Steady-State Error
    
    % Calculate Rise Time: time to reach 90% of final value
    idx_rise = find(y >= 0.9, 1);
    if isempty(idx_rise)
        Tr = t(end);
    else
        Tr = t(idx_rise);
    end
    
    % Calculate Settling Time: time when output remains within 2% of final value
    idx_settle = find(abs(y - 1) <= 0.02, 1, 'last');
    if isempty(idx_settle)
        Ts = t(end);
    else
        Ts = t(idx_settle);
    end
    
    % Weighting parameter (adjust as needed)
    rho = 0.5;  
    
    % Compute the objective function value to be minimized
    F = (1 - exp(rho)) * ((OS / 100) + Ess) + exp(rho) * (Ts - Tr);
end
