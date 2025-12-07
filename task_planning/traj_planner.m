function X_DES = traj_planner(t)

    global BASE_PATH T_TOTAL AMP_SNAKE

    numJoints_snake = 7;
    numJoints_total = 10;  

    if isempty(BASE_PATH) || size(BASE_PATH,1) < 2 || T_TOTAL <= 0
        amplitude  = AMP_SNAKE;
        wavelength = 6.0;
        freq       = 2 * pi * 0.1;

        q_snake_des  = zeros(numJoints_snake, 1);
        dq_snake_des = zeros(numJoints_snake, 1);

        for i = 1:numJoints_snake
            phase_position = (2 * pi * i) / wavelength;
            phase_time     = freq * t;
            phase          = phase_position + phase_time;

            q_snake_des(i)  = amplitude * sin(phase);
            dq_snake_des(i) = -amplitude * freq * cos(phase);
        end

        q_des  = [0; 0; 0; q_snake_des];
        dq_des = [0; 0; 0; dq_snake_des];

        X_DES = [q_des; dq_des];
        return;
    end


    persistent cumLen totalLen
    if isempty(cumLen)
        seg      = sqrt(sum(diff(BASE_PATH).^2, 2));  
        cumLen   = [0; cumsum(seg)];
        totalLen = cumLen(end);
        if totalLen <= 0
            totalLen = 1;
        end
    end

    s_time = min(max(t / T_TOTAL, 0), 1);
    sL     = s_time * totalLen;   

    idx = find(cumLen >= sL, 1, 'first');
    if isempty(idx)
        idx = length(cumLen);
    end

    if idx == 1
        p1 = BASE_PATH(1,:);
        p2 = BASE_PATH(2,:);
        alpha = 0;
    elseif idx >= length(cumLen)
        p1 = BASE_PATH(end-1,:);
        p2 = BASE_PATH(end,:);
        alpha = 1;
    else
        p1 = BASE_PATH(idx-1,:);
        p2 = BASE_PATH(idx,:);
        alpha = (sL - cumLen(idx-1)) / (cumLen(idx) - cumLen(idx-1));
    end

    p = (1-alpha)*p1 + alpha*p2;  

    dirVec = p2 - p1;
    if norm(dirVec) < 1e-6
        yaw = -pi/2;    
        tang = [1; 0];
    else
        theta = atan2(dirVec(2), dirVec(1));
        yaw   = theta + pi/2;  
        tang  = (dirVec / norm(dirVec))';
    end

    v_path = totalLen / T_TOTAL;    
    dp   = v_path * tang;       
    dyaw = 0;                       

    amplitude  = AMP_SNAKE;
    wavelength = 6.0;
    freq       = 2 * pi * 0.1;

    q_snake_des  = zeros(numJoints_snake, 1);
    dq_snake_des = zeros(numJoints_snake, 1);

    for i = 1:numJoints_snake
        phase_position = (2 * pi * i) / wavelength;
        phase_time     = freq * t;
        phase          = phase_position + phase_time;

        q_snake_des(i)  = amplitude * sin(phase);
        dq_snake_des(i) = -amplitude * freq * cos(phase);
    end

    corridor_x_min = -0.5;
    corridor_x_max =  5.5;

    if p(1) >= corridor_x_min && p(1) <= corridor_x_max
        q_snake_des(:)  = 0;
        dq_snake_des(:) = 0;
    end
    q_des  = [p(1); p(2); yaw; q_snake_des];
    dq_des = [dp(1); dp(2); dyaw; dq_snake_des];

    X_DES = [q_des; dq_des];
    if numel(X_DES) ~= 2*numJoints_total
        error('traj_planner: X_DES has wrong length (%d, expected %d).', ...
              numel(X_DES), 2*numJoints_total);
    end
end
