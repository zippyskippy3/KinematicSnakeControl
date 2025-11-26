function X_DES = traj_planner(t)

amplitude = deg2rad(45.0);
wavelength = 4.0;
freq = 2 * pi * 0.5;

q_des = zeros(7, 1);
dq_des = zeros(7, 1);

for i = 1:7
    phase_position = (2 * pi * i) / wavelength;
    phase_time = freq * t;
    phase = phase_position - phase_time;
    
    q_des(i) = amplitude * sin(phase);
    
    dq_des(i) = -amplitude * freq * cos(phase);
end

X_DES = [q_des; dq_des];

end
