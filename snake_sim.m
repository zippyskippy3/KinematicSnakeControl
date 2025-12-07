addpath("task_control")
addpath("task_planning")

clear; clc;


robot = importrobot('./snake_robot/snake_description/urdf/sevenJoints.urdf', ...
                    MeshPath="./snake_robot/snake_description/meshes");
robot.DataFormat = 'column';

numJoints_snake = 7;
numJoints_total = 10;

t_span = [0 60];    

q_snake_0 = zeros(numJoints_snake, 1);


x1 = 0.0;  
x2 = 3.0;   
gap_half   = 2.5;  
wall_thick = 0.7; 

% Bottom wall
bottom_block = [x1 x2 x2 x1; -gap_half -gap_half -(gap_half + wall_thick) -(gap_half + wall_thick)];

% Top wall
top_block = [x1 x2 x2 x1; gap_half gap_half (gap_half + wall_thick) (gap_half + wall_thick)];


x3 = 3.5;
x4 = 5.0;

bottom_block2 = [x3 x4 x4 x3; -gap_half -gap_half -(gap_half + wall_thick) -(gap_half + wall_thick)];
top_block2    = [x3 x4 x4 x3; gap_half gap_half (gap_half + wall_thick) (gap_half + wall_thick)];
obstacle_stack = {bottom_block, top_block, bottom_block2, top_block2};


q_start = [-2.0; 0.0];
q_goal  = [5.5;  0.0];

world_bounds = [-3 7; -4 4];

N_samples = 300;   
k_near    = 12;    
step_vis  = 0.05; 

[V_nodes, E_adj, path_idx] = buildPRM_basic(q_start, q_goal, obstacle_stack, N_samples, k_near, step_vis, world_bounds);

base_path = V_nodes(path_idx, :);   

if isempty(base_path)
    warning('PRM could not find a path. The base will not move along a planned path.');
end


global BASE_PATH T_TOTAL AMP_SNAKE;
BASE_PATH = base_path;       
T_TOTAL   = t_span(end);    

AMP_SNAKE = deg2rad(25.0);     

if ~isempty(BASE_PATH)
    q_base_0 = [BASE_PATH(1,1); BASE_PATH(1,2); 0];
else
    q_base_0 = [q_start(1); q_start(2); 0];
end

q0  = [q_base_0; q_snake_0];   
dq0 = zeros(numJoints_total, 1);
X0  = [q0; dq0];              


[T, X] = ode45(@(t, x) snakeDynamics(t, x, robot, numJoints_total), t_span, X0);

figure;
plot(T, rad2deg(X(:, 4:3+numJoints_snake)));
title('Snake Joint Positions Over Time');
xlabel('Time (s)');
ylabel('Joint Angle (Degrees)');
legend(arrayfun(@(i) sprintf('Joint %d', i), 1:numJoints_snake, 'UniformOutput', false), ...
       'Location', 'southeast');
grid on;


figure;
axis equal;
hold on;
title('Serpentine Locomotion with Corridor Obstacles & PRM Path');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
view(3);

ground_size = 8;
patch([-ground_size, ground_size, ground_size, -ground_size], [-ground_size, -ground_size, ground_size, ground_size], [-0.2, -0.2, -0.2, -0.2], [0.8 0.8 0.8], 'FaceAlpha', 0.5);

% Obstacles
for i = 1:numel(obstacle_stack)
    poly = obstacle_stack{i};
    px = poly(1,:);
    py = poly(2,:);

    z_bottom = -0.2 * ones(size(px));
    z_top    =  0.3 * ones(size(px));

    % Top and bottom faces
    fill3(px, py, z_bottom, [1 0 0],   'FaceAlpha', 0.6, 'EdgeColor', 'none');
    fill3(px, py, z_top,    [0.8 0 0], 'FaceAlpha', 0.6, 'EdgeColor', 'none');

    % Side faces
    for k = 1:numel(px)
        k2 = mod(k, numel(px)) + 1;
        xs = [px(k)  px(k2) px(k2) px(k)];
        ys = [py(k)  py(k2) py(k2) py(k)];
        zs = [-0.2   -0.2   0.3    0.3];
        fill3(xs, ys, zs, [0.9 0.2 0.2], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
end


if ~isempty(base_path)
    plot3(base_path(:,1), base_path(:,2), ...
          0.01*ones(size(base_path,1),1), 'LineWidth', 3);
end

player = show(robot, X(1, 1:numJoints_total)', 'Visuals','on');
for k = 1:size(X, 1)
    show(robot, X(k, 1:numJoints_total)', 'PreservePlot', false, 'FastUpdate', true);
    drawnow;
end
hold off;

function dx = snakeDynamics(t, x, robot, numJoints_total)

    q  = x(1:numJoints_total);
    dq = x(numJoints_total+1:end);

    M_full    = massMatrix(robot, q);
    G_full    = gravityTorque(robot, q);
    C_dq_full = velocityProduct(robot, q, dq);

    X_DES = traj_planner(t);

    tau_full = PD_FL_controller(robot, t, x, X_DES, M_full, G_full, C_dq_full);

    tau_friction = calculate_friction_torque(robot, q, dq);

    ddq = M_full \ (tau_full - G_full - C_dq_full + tau_friction);

    dx = [dq; ddq];
end


function [V, E, path_idx] = buildPRM_basic(q_start, q_goal, obstacles, N_samples, k_near, step_vis, bounds)
   
    samples = zeros(N_samples, 2);
    n_valid = 0;
    while n_valid < N_samples
        q_rand = [ ...
            bounds(1,1) + (bounds(1,2)-bounds(1,1))*rand(); ...
            bounds(2,1) + (bounds(2,2)-bounds(2,1))*rand() ];
        if Feasible_base(q_rand, obstacles)
            n_valid = n_valid + 1;
            samples(n_valid,:) = q_rand';
        end
    end

   
    V = [q_start'; q_goal'; samples];
    nV = size(V,1);

   
    E = cell(nV,1);

    D = zeros(nV);
    for i = 1:nV
        for j = i+1:nV
            d = norm(V(i,:) - V(j,:));
            D(i,j) = d;
            D(j,i) = d;
        end
    end

    for i = 1:nV
        [~, idx_sorted] = sort(D(i,:));
        neighbors = idx_sorted(2 : min(k_near+1, nV));  

        for j = neighbors
            if i == j
                continue;
            end
            if Visible_base(V(i,:)', V(j,:)', obstacles, step_vis)
                cost = D(i,j);
                E{i} = [E{i}; j, cost];
                E{j} = [E{j}; i, cost];  
            end
        end
    end

    start_idx = 1;
    goal_idx  = 2;
    path_idx  = dijkstra_shortest_path(E, start_idx, goal_idx);
end

function ok = Feasible_base(q, obstacles)

    x = q(1); y = q(2);
    for i = 1:numel(obstacles)
        poly = obstacles{i};
        px = poly(1,:); py = poly(2,:);
        inside = inpolygon(x, y, px, py);
        if inside
            ok = false;
            return;
        end
    end
    ok = true;
end

function ok = Visible_base(q1, q2, obstacles, step)
  
    diff = q2 - q1;
    dist = norm(diff);
    if dist < 1e-6
        ok = true;
        return;
    end
    n_steps = max(2, ceil(dist/step));
    for s = linspace(0,1,n_steps)
        q = q1 + s*diff;
        if ~Feasible_base(q, obstacles)
            ok = false;
            return;
        end
    end
    ok = true;
end

function path_idx = dijkstra_shortest_path(E, start_idx, goal_idx)

    nV = numel(E);
    dist    = inf(nV,1);
    prev    = zeros(nV,1);
    visited = false(nV,1);

    dist(start_idx) = 0;

    for iter = 1:nV
     
        unvisited = find(~visited);
        if isempty(unvisited)
            break;
        end
        [~, k] = min(dist(unvisited));
        u = unvisited(k);

        if isinf(dist(u))
            break;  
        end

        visited(u) = true;
        if u == goal_idx
            break;
        end

        neighbors = E{u};
        for m = 1:size(neighbors,1)
            v = neighbors(m,1);
            w = neighbors(m,2);
            if visited(v)
                continue;
            end
            alt = dist(u) + w;
            if alt < dist(v)
                dist(v) = alt;
                prev(v) = u;
            end
        end
    end

    % Reconstruct path
    if isinf(dist(goal_idx))
        path_idx = [];
        return;
    end

    path_idx = goal_idx;
    u = goal_idx;
    while u ~= start_idx
        u = prev(u);
        path_idx = [u; path_idx];
    end
end

