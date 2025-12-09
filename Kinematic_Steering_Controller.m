function gamma = Kinematic_Steering_Controller(x, x_des)
    
    % --- Controller Gains (These are the steering gains) ---
    K_y = 0.5;   % Proportional gain for lateral position error (e_y)
    K_theta = 2.0; % Proportional gain for heading angle error (e_theta)
    
    % --- State Extraction (from 20-element vector x) ---
    theta = x(8);   % Current head angle
    py = x(10);     % Current lateral position
    
    % --- Reference Extraction (from X_DES) ---
    theta_des = x_des(8); % Desired head angle
    py_des = 0;           % Target path is the x-axis
    
    % --- Error Calculation ---
    e_y = py - py_des;          
    e_theta = theta - theta_des; 

    % --- Calculate Steering Angle Gamma ---
    % Steering angle is a proportional blend of lateral error and heading error
    gamma = -K_y * e_y - K_theta * e_theta;
    
    % --- Saturation (Crucial to prevent wild steering) ---
    MAX_GAMMA = deg2rad(15); % Max 15 degrees steering offset
    gamma = max(min(gamma, MAX_GAMMA), -MAX_GAMMA);
end