function X_DES = traj_planner(t, x)

% body angle desired

alpha = pi / 6;
delta = 36 * pi / 180;
eta = 70 * pi * t / 180;
deta = 70 * pi / 180;

q_des = zeros(7, 1);
dq_des = zeros(7, 1);

for i = 1:7
    q_des(i) = alpha * sin(eta + (i-1)*delta) + phi_0;
    dq_des(i) = -alpha * cos(eta + (i-1)*delta) * deta;
end

% head angle desired

% design parameter look ahead distance
LAD = 1.4;

py = x(10);

% aligned with the x axis as the goal path
py_des = 0;

theta_des = -atan(py_des - py / LAD);

X_DES = [q_des theta_des; dq_des dtheta_des];


end

