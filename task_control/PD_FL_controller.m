function u = PD_FL_controller(robot, t, x, x_des)

    q = x(1:3);
    dq = x(4:6);

    q_des = x_des(1:3);
    dq_des = x_des(4:6);

    Kp = diag([9, 9, 9]);  % Proportional gain
    Kd = diag([6, 6, 6]);     % Derivative gain
    
    M = massMatrix(robot, q);
    G = gravityTorque(robot, q);
    C = velocityProduct(robot, q, dq);

    u =  (C * dq + G) + M * ( - Kp * scale_q_to_pi(q - q_des) - Kd * (dq - dq_des) );
    
end

