robot_urdf_file = './snake_robot/snake_description/urdf/threeJoints.urdf'; 
gravity_vector = [0; 0; -9.81];

robot = importrobot(robot_urdf_file, MeshPath="./snake_robot/snake_description/meshes");
show(robot,Visuals="on",Collisions="off", Frames="off");

robot.DataFormat = 'column';

q = randomConfiguration(robot);

q_dot = zeros(size(q));

M