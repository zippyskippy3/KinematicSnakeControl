function u = PD_FL_controller(robot, t, x, x_des, M_full, G_full, C_dq_full)
    % The full state x is 20x1 (10 pos, 10 vel)
    % The desired state x_des is 14x1 (7 pos, 7 vel)
    
    numJoints_total = 10;
    numJoints_snake = 7;
    idx_snake = 4:10; % Indices for the snake joints (q4 through q10)

    % 1. Extract current internal state
    q = x(idx_snake);    
    dq = x(idx_snake + numJoints_total); 
    
    % 2. Extract