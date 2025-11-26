addpath("task_control")
addpath("task_planning")

clear; clc;

% Import the robot URDF
robot = importrobot('./snake_robot/snake_description/urdf/sevenJoints.urdf', MeshPath="./snake_robot/snake_description/meshes");

% Display for debugging
% show(robot,Visuals="on",Collisions="off", Frames="off");

robot.DataFormat = 'column';

numJoints = 7;

t_span = [0 15];

amp = deg2rad(45);
q_snake_0 = zeros(7, 1);

for i = 1:7
    phase_position = (2 * pi * i) / 4;
    
    q_snake_0(i) = amp * sin(phase_position);
end

q_base_0 = [0; 0; 0]; 
q0 = [q_base_0; q_snake_0];
dq0 = zeros(10, 1); 
X0 = [q0; dq0];

[T, X] = ode45(@(t, x) snakeDynamics(t, x, robot, 10), t_span, X0);

figure;
plot(T, rad2deg(X(:, 1:numJoints)));
title('Joint Positions Over Time');
xlabel('Time (s)');
ylabel('Joint Angle (Degrees)');
legend(arrayfun(@(i) sprintf('Joint %d', i), 1:numJoints, 'UniformOutput', false), 'Location', 'southeast');
grid on;

% ground plane visualization
figure;
axis equal;
hold on;
title('Serpentine Locomotion Simulation');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
view(3);

ground_size = 5;
patch([-ground_size, ground_size, ground_size, -ground_size], ...
      [-ground_size, -ground_size, ground_size, ground_size], ...
      [-0.2, -0.2, -0.2, -0.2], [0.8 0.8 0.8], 'FaceAlpha', 0.5);

player = show(robot, X(1, 1:numJoints+3)', 'Visuals','on');
for k = 1:size(X, 1)
    show(robot, X(k, 1:numJoints+3)', 'PreservePlot', false, 'FastUpdate', true);
    drawnow;
end
hold off;

function dx = snakeDynamics(t, x, robot, numJoints_total) 
    
    q = x(1:numJoints_total); 
    dq = x(numJoints_total+1:end); 

    M_full = massMatrix(robot, q);
    G_full = gravityTorque(robot, q);
    C_dq_full = velocityProduct(robot, q, dq);
    
    X_DES = traj_planner(t); 

    tau_pd_internal = PD_FL_controller(robot, t, x, X_DES, M_full, G_full, C_dq_full); 
    
    tau_control = [zeros(3, 1); tau_pd_internal]; 
    
    tau_friction = calculate_friction_torque(robot, q, dq);
    
    ddq = M_full \ (tau_control - G_full - C_dq_full + tau_friction);
    
    dx = [dq; ddq]; 
end