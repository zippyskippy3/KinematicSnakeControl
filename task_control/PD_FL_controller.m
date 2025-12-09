% % Applies control to the internal joint angles and to phi_0
% % Theta, px, and py are outcomes of the control due to the state
% % eta is included in the state because it takes the time dependancy out
% % of the reference joint trajectories
% 
% function u = PD_FL_controller(robot, t, x, x_des, M_full, G_full, C_dq_full)
% 
%     % Control of the body joint angles
% 
%     N = 8;
% 
%     q_a = x(1:N-1);    
%     dq_a = x(13:19); 
% 
%     theta = x(8);
%     dtheta = x(20);
% 
%     phi_0 = x(11);
%     dphi_0 = x(24);
% 
%     q_a_des = x_des(1:N-1); 
%     dq_a_des = x_des(9:15); 
% 
%     theta_des = x_des(8); 
%     dtheta_des = x_des(20);
% 
%     phi_0_des = 0;
%     dphi_0_des = 0;
% 
%     Kp = diag(ones(1, N-1) * 5); 
%     Kd = diag(ones(1, N-1) * 0.5);
% 
%     ddq_a_des = - Kp * scale_q_to_pi(q_a - q_a_des) - Kd * (dq_a - dq_a_des); 
% 
%     % control of the head angle (theta)
% 
%     M_11 = M_full(1:N-1, 1:N-1); 
%     M_22 = M_full(N:end, N:end);
%     M_21 = M_full(N:end, 1:N-1);
%     M_12 = M_full(1:N-1, N:end);
% 
%     h1 = (C_dq_full(1:N-1) + G_full(1:N-1));
%     h2 = (C_dq_full(N:end) + G_full(N:end));
% 
%     Beta = (-pinv(M_22)*M_21);
%     f = -(pinv(M_22)*h2);
%     f_theta = f(1);
% 
%     Kp_theta = 20;
%     kd_theta = 1;
% 
%     ddq_u_des = -pinv(M_22) * (h2 + M_21 * ddq_a_des);
% 
%     ddtheta_des = ddq_u_des(1);
% 
%     ddphi_0 = (-f_theta + ddtheta_des - Kp_u * (theta - theta_des) - Kd_u * (dtheta - dtheta_des) - Kp_phi * (phi_0 - phi_0_des) - Kd_phi * (dphi_0 - dphi_0_des)) / sum(Beta);
% 
%     u = (M_11 - M_12 * M_22_inv * M_21) * (ddphi_0-ddq_a_des) * (h1 - M_12 * M_22_inv * h2);

% 
% end

function [u, ddphi_0] = PD_FL_controller(robot, t, x, x_des, M_full, G_full, C_dq_full)
   
    N_JOINTS = 7;
    N_AUGMENTED = 12; % Indices 1-12 are positions (q_a, theta, px, py, eta, phi_0)
    
    % --- 1. State & Reference Extraction ---
    q_a = x(1:N_JOINTS);       
    dq_a = x(N_AUGMENTED+1 : N_AUGMENTED+N_JOINTS); 
    theta = x(N_JOINTS+1);     % x(8)
    dtheta = x(N_AUGMENTED+N_JOINTS+1); % x(20)
    phi_0 = x(N_AUGMENTED);    % x(12)
    dphi_0 = x(N_AUGMENTED+N_AUGMENTED); % x(24)
    
    % Reference Extraction from traj_planner
    q_a_des = x_des(1:N_JOINTS);
    theta_des = x_des(N_JOINTS+1); 
    dq_a_des = x_des(N_JOINTS+2 : 2*N_JOINTS+1);
    dtheta_des = x_des(2*N_JOINTS+2);
    ddq_a_feedforward = x_des(2*N_JOINTS+3 : 3*N_JOINTS+2);
    ddtheta_feedforward = x_des(3*N_JOINTS+3); 
    
    % --- 2. Gains and Errors ---
    Kp = diag(ones(1, N_JOINTS) * 10); 
    Kd = diag(ones(1, N_JOINTS) * 2);
    Kp_u = 10; Kd_u = 2;       % Gains for theta (path following)
    Kp_phi = 10; Kd_phi = 2;    % Gains for phi_0 stabilization
    
    phi_0_des = 0;           
    dphi_0_des = 0;          
    
    % --- 3. Matrix Partitioning and Intermediates ---
    % M_full is 10x10. Actuated (7, q_a) and Unactuated (3, theta, px, py)
    M_11 = M_full(1:N_JOINTS, 1:N_JOINTS);      % 7x7 (Actuated-Actuated)
    M_22 = M_full(N_JOINTS+1:end, N_JOINTS+1:end);% 3x3 (Unactuated-Unactuated)
    M_21 = M_full(N_JOINTS+1:end, 1:N_JOINTS);   % 3x7
    M_12 = M_full(1:N_JOINTS, N_JOINTS+1:end);   % 7x3
    
    h1 = (C_dq_full(1:N_JOINTS) + G_full(1:N_JOINTS)); % 7x1
    h2 = (C_dq_full(N_JOINTS+1:end) + G_full(N_JOINTS+1:end)); % 3x1
    
    M_22_inv = pinv(M_22); % Use pinv for robustness
    
    % Effective Dynamics Matrices (Eq. 17)
    M_hat = M_11 - M_12 * M_22_inv * M_21; % M_hat_11 (7x7)
    H_hat = h1 - M_12 * M_22_inv * h2;     % H_hat_1 (7x1)
    
    % Dynamics for Unactuated Variables (Eq. 16)
    Beta = -M_22_inv * M_21; % 3x7
    f = -M_22_inv * h2;      % 3x1
    
    % --- 4. Dynamic Compensator (ddphi_0) ---
    % Path Following Law is applied to the first unactuated component (theta)
    % The constraint is theta_c = theta + Beta_vector * q_a = phi_0
    
    % We use the theta (first unactuated row) from Beta and f.
    Beta_vector = Beta(1, :); % 1x7: Relates theta acc (first row) to tau_a
    f_theta = f(1);           % Scalar: Remainder for theta acc
    
    % Path following virtual control (v_theta, based on Eq. 18's second line)
    v_theta = ddtheta_feedforward - Kp_u * (theta - theta_des) - Kd_u * (dtheta - dtheta_des);
    
    v_phi = 0 - Kp_phi * (phi_0 - phi_0_des) - Kd_phi * (dphi_0 - dphi_0_des);
    
    ddq_a_feedback = ddq_a_feedforward - Kp * (q_a - q_a_des) - Kd * (dq_a - dq_a_des);
    
    ddphi_0 = (v_theta - f_theta - Beta_vector * ddq_a_feedback) / sum(Beta_vector);
    
    vartheta = ddq_a_feedback + ones(N_JOINTS, 1) * ddphi_0; 

    u = M_hat * vartheta + H_hat;

end