function u = PD_FL_controller(robot, t, x, x_des, M_full, G_full, C_dq_full)
    
    numJoints_total = 10;
    numJoints_snake = 7;
    idx_snake = 4:10; 

    q = x(idx_snake);    
    dq = x(idx_snake + numJoints_total); 
    
    q_des = x_des(1:numJoints_snake); 
    dq_des = x_des(numJoints_snake+1:end); 
    
    M_snake = M_full(idx_snake, idx_snake); 
    G_snake = G_full(idx_snake);            
    C_dq_snake = C_dq_full(idx_snake);      

    Kp = diag(ones(1, 7) * 5); 
    Kd = diag(ones(1, 7) * 0.5);
    
    ddq_des = - Kp * scale_q_to_pi(q - q_des) - Kd * (dq - dq_des); 
    
    u = (C_dq_snake + G_snake) + M_snake * ddq_des; 
end