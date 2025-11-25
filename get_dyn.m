robot_urdf_file = './snake_robot/snake_description/urdf/threeJoints.urdf'; 
gravity_vector = [0; 0; -9.81];

robot = importrobot(robot_urdf_file);
show(robot,Visuals="on",Collisions="off");

robot.DataFormat = 'column';

q = randomConfiguration(robot);

q_dot = zeros(size(q));

M_matrix = massMatrix(robot, q);

G = gravityTorque(robot, q);

C = velocityProduct(robot, q, q_dot);