function X_DES = traj_planner(t)
NUM_JOINTS = 7;
JOINT_INDICES = 1:NUM_JOINTS;

AMPLITUDE_DEG = 45.0;
AMPLITUDE_RAD = deg2rad(AMPLITUDE_DEG);
WAVELENGTH_SEGMENTS = 4.0;
FREQUENCY_HZ = 0.5;
ANGULAR_FREQUENCY = 2 * pi * FREQUENCY_HZ;

q_des = zeros(NUM_JOINTS, 1);
dq_des = zeros(NUM_JOINTS, 1);

for i = JOINT_INDICES
    phase_position = (2 * pi * i) / WAVELENGTH_SEGMENTS;
    phase_time = ANGULAR_FREQUENCY * t;
    phase = phase_position - phase_time;
    
    q_des(i) = AMPLITUDE_RAD * sin(phase);
    
    dq_des(i) = -AMPLITUDE_RAD * ANGULAR_FREQUENCY * cos(phase);
end

X_DES = [q_des; dq_des];

end
