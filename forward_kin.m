function [dpx, dpy, ddpx, ddpy] = forward_kin(robot, x)
    
    % Function that calculates the x and y of the center of mass 
    % Will need to use jacobian to get the velocity and acceleration

    q = x(1:7);
    dq = x(13:19);

    [J_cm, dJ_cm_dq] = jacobianCenterOfMass(robot, q_rbt);

    J_cm_linear = J_cm(1:3, :);
    dJ_cm_linear_dq = dJ_cm_dq(1:3, :);

    a_cm_full = J_cm_linear * ddq_rbt + dJ_cm_linear_dq * dq_rbt;

    V_cm_spatial = J_cm * dq;
    v_cm_linear = V_cm_spatial(4:6);
    dpx = v_cm_linear(1);
    dpy = v_cm_linear(2);

    ddpx = a_cm_full(1);
    ddpy = a_cm_full(2);
end