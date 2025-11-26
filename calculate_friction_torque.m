function tau_friction = calculate_friction_torque(robot, q, dq)

    mu_t = 0.1;  
    mu_n = 1.0;  
    g = 9.81; 
    
    tau_friction = zeros(10, 1);
    
    bodyNames = robot.BodyNames(2:end);
    
    tau_friction_vector = zeros(10, 1);
    
    for i = 1:7

        body_name = bodyNames{i};
        body = getBody(robot, body_name);
        
        T_body = getTransform(robot, q, body_name);
        R_body = T_body(1:3, 1:3); 
        
        J_CoM = geometricJacobian(robot, q, body_name);
        v_CoM = J_CoM * dq;
        v_linear = v_CoM(1:3);
        
        e_t = R_body * [1; 0; 0]; 
        e_n = R_body * [0; 1; 0]; 
        

        v_t = dot(v_linear, e_t) * e_t; 
        v_n = dot(v_linear, e_n) * e_n; 
       
        m = body.Mass; 
        N = m * g;    
        F_friction = (mu_t * N) * sign(v_t) + (mu_n * N) * sign(v_n);

        J_CoM_linear = J_CoM(1:3, :); 
        tau_friction_link = J_CoM_linear' * F_friction;
        
        tau_friction_vector = tau_friction_vector + tau_friction_link;
    end
    
    tau_friction = tau_friction_vector;

end