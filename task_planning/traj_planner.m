function X_DES = traj_planner(t, x)

% Internal body angles desired

T = 2;

alpha = deg2rad(45.0);
delta = (2 * pi) / 6.0;
eta = x(11);
phi_0 = x(12);
dphi_0 = x(24);
deta = 0.2 * pi / T;

q_des = zeros(7, 1);
dq_des = zeros(7, 1);
ddq_des = zeros(7, 1);

% Requires this second degree because it is a second degree Lie
% derivative that reveals the control variables

for i = 1:7
    q_des(i) = alpha * sin(eta + (i-1)*delta) + phi_0;
    dq_des(i) = -alpha * cos(eta + (i-1)*delta) * deta + dphi_0;
    ddq_des(i) = -alpha * sin(eta + (i-1)*delta) * deta^2;
end

% Design parameter look ahead distance
LAD = 0.5;

py = x(10);
dpy = x(22);

% Aligned with the x axis as the goal path
py_des = 0;

% Head angle desired
theta_des = -atan((py_des - py) / LAD);
dtheta_des = -(1 / (1 + ((py_des - py) / LAD)^2)) * (-dpy / LAD);
ddtheta_des = 0;

X_DES = [q_des; theta_des; dq_des; dtheta_des; ddq_des; ddtheta_des];

end

