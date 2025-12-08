function [x_sol,u_sol] = task_trajopt(period_opt, N, q_init, q_target, x_init)

        import casadi.*

        % Define snake joint dimension
        numJoints = 7; 
        
        dq_max = 0.2; dq_min = -dq_max;
        q_max = 2*pi;
        q_min = -2*pi;
        
        % Symbolic variables for 7 joints
        states = SX.sym('q', numJoints);
        n_states = length(states); % n_states = 7
        
        controls = SX.sym('dq', numJoints);
        n_controls = length(controls); % n_controls = 7
        
        rhs = controls; % system r.h.s: Assuming U is desired joint velocity
        
        f = Function('f',{states,controls},{rhs}); 
        U = SX.sym('U',n_controls,N);
        P = SX.sym('P',n_states,2);
        
        X = SX.sym('X',n_states,N+1);
        
        obj = 0; 
        g = [];  
        
        % =================================================================
        % 1. CASADI SYMBOLIC KINEMATICS FUNCTION CREATION
        % This creates the required 'snake_kin_casadi' function
        % =================================================================
        L = 0.1; % Link length (MUST match your robot's physical link length)
        q_kin_sym = SX.sym('q_kin_sym', numJoints); 

        p_x = SX(0);
        p_y = SX(0);
        abs_angle = SX(0);

        % R_points_list will store the [x; y] coordinates of all 8 joint centers
        R_points_list = { [p_x; p_y] }; % Point 1: Base (0, 0)

        for i = 1:numJoints
            % Absolute angle of the link segment relative to the world X-axis
            abs_angle = abs_angle + q_kin_sym(i); 
            
            % Forward Kinematics: Add the projection of the link length L
            p_x = p_x + L * cos(abs_angle);
            p_y = p_y + L * sin(abs_angle);
            
            % Store the new joint center (Point i+1)
            R_points_list{end+1} = [p_x; p_y];
        end

        % Concatenate all 8 symbolic points into a 2x8 matrix
        R_points_SX = [R_points_list{:}];

        % Create the final symbolic function
        snake_kin_casadi = Function('snake_kin_casadi', {q_kin_sym}, {R_points_SX});
        % =================================================================
        
        % Weighing matrices (7x7)
        Q = diag(ones(numJoints, 1) * 1); 
        R = diag(ones(numJoints, 1) * 0.05);
        Q_f = diag(ones(numJoints, 1) * 500);
        
        st  = X(:,1); 
        g = [g;st-P(:,1)]; 
        
        for k = 1:N
            st = X(:,k);  con = U(:,k);
            obj = obj+(st-P(:,2))'*Q*(st-P(:,2)) + con'*R*con; 
            st_next = X(:,k+1);
            f_value = f(st,con);
            st_next_euler = st+ (period_opt*f_value);
            g = [g;st_next-st_next_euler]; 
        end
        st = X(:,N+1);
        obj = obj + (st-P(:,2))'*Q_f*(st-P(:,2));
        
        % Add constraints for collision avoidance
        obs = [1.5 -2; 0.0 0.0]; 
        obs_r = [0.5; 0.5]; 

        % R_points is the 2x8 matrix [x; y] of all collision points
        num_collision_points = 8; 

        for k = 1:N+1
            % --- 2. FUNCTION CALL (Integration) ---
            % X(:,k) is the symbolic state vector (q) for this time step.
            R_points = snake_kin_casadi(X(:,k)); 
            
            for i = 1:size(obs,2) % Iterate through obstacles
                obs_center = obs(:, i);
                obs_radius = obs_r(i);

                for j = 1:num_collision_points % Check 8 points (Base to J7)
                    point = R_points(:, j);
                    % Collision constraint: Distance^2 - Radius^2 >= 0 
                    g = [g ; - ((point(1)-obs_center(1))^2 + (point(2)-obs_center(2))^2 - obs_radius^2)];
                end
            end
            
            % Table constraints (assuming R_points(2, :) is the vertical height y)
            for j = 1:num_collision_points
                g = [g ; - R_points(2, j)]; 
            end
        end

        % Calculate total number of inequality constraints
        num_ineq_constraints = size(obs, 2) * num_collision_points * (N+1) + num_collision_points * (N+1);

        % make the decision variable one column vector
        OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];
        
        nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
        
        opts = struct;
        opts.ipopt.print_level =3;
        opts.print_time = 0;
        opts.ipopt.acceptable_tol =1e-6;
        opts.ipopt.acceptable_obj_change_tol = 1e-4;
        
        solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
        
        args = struct;
        num_dyn_constraints = n_states*(N+1);
        
        args.lbg(1:num_dyn_constraints) = 0; % equality constraints
        args.ubg(1:num_dyn_constraints) = 0; % equality constraints
        
        args.lbg(num_dyn_constraints+1 : num_dyn_constraints + num_ineq_constraints) = -inf; % inequality constraints
        args.ubg(num_dyn_constraints+1 : num_dyn_constraints + num_ineq_constraints) = 0; % inequality constraints
        
        % Set state bounds (q_min/q_max) for all 7 states
        for i = 1:n_states
            idx_state = i : n_states : n_states*(N+1);
            args.lbx(idx_state, 1) = q_min; 
            args.ubx(idx_state, 1) = q_max;
        end
        
        % Set control bounds (dq_min/dq_max) for all 7 controls
        for i = 1:n_controls
            idx_control = n_states*(N+1) + i : n_controls : n_states*(N+1) + n_controls*N;
            args.lbx(idx_control, 1) = dq_min; 
            args.ubx(idx_control, 1) = dq_max;
        end
        
        args.p   = [q_init q_target]; 
        
        % Set initial guess sizes
        X_init = reshape(x_init, n_states*(N+1), 1);
        U_init = reshape((x_init(:,2:end)-x_init(:,1:end-1))/period_opt, n_controls*N, 1);
        args.x0  = [X_init;U_init];
        
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
             'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        
        x_sol = reshape(full(sol.x(1:n_states*(N+1)))', n_states, N+1)';
        u_sol = reshape(full(sol.x(n_states*(N+1)+1:end))', n_controls, N)';
end