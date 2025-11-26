function u = PD_FL_controller(robot, t, x, x_des, M_full, G_full, C_dq_full)
    % The full state x is 20x1 (10 pos, 10 vel)
    % The desired state x_des is 14x1 (7 pos, 7 vel)
    
    numJoints_total = 10;
    numJoints_snake = 7;
    idx_snake = 4:10; % Indices for the snake joints (q4 through q10)

    % 1. Extract current internal state
    q = x(idx_snake);    
    dq = x(idx_snake + numJoints_total); 
    
    % 2. Extract desired internal state
    q_des = x_des(1:numJoints_snake); % 7x1
    dq_des = x_des(numJoints_snake+1:end); % 7x1
    
    % 3. Slicing the full dynamics matrices
    M_snake = M_full(idx_snake, idx_snake); % 7x7 Mass Matrix block
    G_snake = G_full(idx_snake);            % 7x1 Gravity vector
    C_dq_snake = C_dq_full(idx_snake);      % 7x1 Coriolis vector

    % 4. Define 7x7 gains (use tuned, higher values)
    Kp = diag(ones(1, 7) * 5); 
    Kd = diag(ones(1, 7) * 0.5);
    
    % 5. Calculate the desired acceleration vector (ddq_des)
    ddq_des = - Kp * scale_q_to_pi(q - q_des) - Kd * (dq - dq_des); 
    
    % 6. Apply the Feedback Linearization Control Law (u is 7x1)
    u = (C_dq_snake + G_snake) + M_snake * ddq_des; 
end