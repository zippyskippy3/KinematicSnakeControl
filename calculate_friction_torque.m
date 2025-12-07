function tau_friction = calculate_friction_torque(robot, q, dq)
    
    % Viscous Friction Coefficients (Section II-B)
    % c_n MUST be significantly greater than c_t for propulsion (Property 1) [cite: 1492]
    c_t = 0.5;  % Tangential (forward) viscous friction coefficient
    c_n = 20;  % Normal (sideways) viscous friction coefficient
    
    % Initialize total friction torque vector (size: total DoFs)
    tau_friction_vector = zeros(length(dq), 1);
    
    % Loop through each link (assuming links are 1 to N)
    for i = 1:robot.NumBodies
        body_name = robot.BodyNames{i};
        body = robot.getBody(body_name);
        
        % 1. Get the Jacobian (J_CoM) and World Velocity (v_CoM)
        J_CoM = geometricJacobian(robot, q, body_name);
        v_CoM = J_CoM * dq;
        
        % World Frame Linear Velocity (x, y, z)
        v_linear_world = v_CoM(4:6);
        %disp(v_linear_world);
        
        % 2. Get the Link's Absolute Angle (Theta_i)
        % For a planar snake, the link's absolute angle (theta_i) is the Z-axis rotation.
        % We need the 2D rotation matrix R_i to get the link angle.
        T_body = getTransform(robot, q, body_name);
        
        % Extract the yaw (theta_i) from the transformation matrix
        % T_body(1:3, 1:3) is the rotation matrix R. theta = atan2(R(2,1), R(1,1)).
        theta_i = atan2(T_body(2,1), T_body(1,1));
        
        % Convert velocity to 2D World-frame for the calculation (ignore Z)
        dot_x_i = v_linear_world(1);
        dot_y_i = v_linear_world(2);
        
        % 3. Calculate Friction Matrix Components (Eq. 2a - 2c) [cite: 1453, 1454, 1455]
        cos_sq = cos(theta_i)^2;
        sin_sq = sin(theta_i)^2;
        sin_cos = sin(theta_i) * cos(theta_i);
        
        F_x  = c_t * cos_sq + c_n * sin_sq;
        F_xy = (c_t - c_n) * sin_cos;
        F_y  = c_t * sin_sq + c_n * cos_sq;
        
        % 4. Construct the Friction Matrix (F_matrix)
        F_matrix = [F_x, F_xy; F_xy, F_y];
        
        % 5. Calculate the Friction Force (f_i) in the World Frame (Eq. 1) [cite: 1449]
        % f_i = -F_matrix * [dot_x_i; dot_y_i]
        f_i_xy = -F_matrix * [dot_x_i; dot_y_i];
        
        % 6. Apply Friction Force as Torque
        % The force must be a 3D vector for the 6xN Jacobian transpose
        F_friction_3D = [f_i_xy(1); f_i_xy(2); 0]; 
        
        % Extract the linear part of the Jacobian (J_CoM_linear is 3xN)
        J_CoM_linear = J_CoM(4:6, :);
        
        % tau_friction_link = J_CoM_linear' * F_friction_3D
        tau_friction_link = J_CoM_linear' * F_friction_3D;
        
        
        tau_friction_vector = tau_friction_vector + tau_friction_link;
    end
    
    tau_friction = tau_friction_vector;
end