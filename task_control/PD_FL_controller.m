function u = PD_FL_controller(robot, t, x, x_des, M_full, G_full, C_dq_full)

    numJoints_total = 10;
    numJoints_snake = 7;
    idx_base  = 1:3;
    idx_snake = 4:10;

    q  = x(1:numJoints_total);
    dq = x(numJoints_total+1:end);

    q_des  = x_des(1:numJoints_total);
    dq_des = x_des(numJoints_total+1:end);

    Kp_base  = diag([20 20 10]);  
    Kd_base  = diag([5 5 2]);

    Kp_snake = diag(ones(1, numJoints_snake) * 5);
    Kd_snake = diag(ones(1, numJoints_snake) * 0.5);

    Kp = blkdiag(Kp_base, Kp_snake);
    Kd = blkdiag(Kd_base, Kd_snake);

    q_err  = q - q_des;
    dq_err = dq - dq_des;

    angle_idx = [3, idx_snake];
    if exist('scale_q_to_pi', 'file') == 2
        q_err(angle_idx) = scale_q_to_pi(q_err(angle_idx));
    end

    ddq_des = - Kp * q_err - Kd * dq_err;

    u = C_dq_full + G_full + M_full * ddq_des;
end
