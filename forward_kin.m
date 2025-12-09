function [dpx, dpy, ddpx, ddpy] = forward_kin(robot, x, ddq)
    
    % Assuming the full system has 10 generalized coordinates
    N_DOF = 10;
    
    % --- 1. State Extraction ---
    q = x(1:10); 
    dq = x(15:24); 
    
    % --- 2. Kinematic Calculations ---
    [~, J_cm] = centerOfMass(robot, q);
    
    % J_cm is 3x10 (Linear CM Jacobian only)
    J_cm_linear = J_cm; % FIX: Use the entire 3x10 matrix
    
    % --- 3. Placeholder for Jacobian Derivative ---
    dJ_cm_linear_dq = zeros(3, N_DOF); 
    
    % --- 4. Acceleration Calculation ---
    a_cm_full = J_cm_linear * ddq + dJ_cm_linear_dq * dq;
    
    % --- 5. Velocity Calculation (Must be re-evaluated!) ---
    % Since J_cm is only 3x10, the velocity calculation must use J_cm_linear
    V_cm_linear = J_cm_linear * dq; % Calculate linear velocity directly
    
    dpx = V_cm_linear(1);
    dpy = V_cm_linear(2);
    
    ddpx = a_cm_full(1);
    ddpy = a_cm_full(2);
    
end