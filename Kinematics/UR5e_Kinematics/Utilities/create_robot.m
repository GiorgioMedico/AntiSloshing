function robot = create_robot(robot_type)

filepath = ['irma_',robot_type,'_calibration.xml'];
robot = readstruct(filepath);
robot = rmfield(robot,{'hash'});
fields = fieldnames(robot);

n_joints = length(fields);
Mxyz_rpy = zeros(6,length(fields));
r = zeros(3,1);
Rot = eye(3);
for i = 1:n_joints
    
    struct_xyz_rpy = eval(['robot.', fields{i}]);
    table_xyz_rpy = struct2table(struct_xyz_rpy);
    Mxyz_rpy(:,i) = table2array(table_xyz_rpy)';
    
    r = r + Rot*Mxyz_rpy(1:3,i);
    Rot = Rot*eul2rotm(Mxyz_rpy(4:6,i)','XYZ');       
    
    A(:,:,i) = Rot;
    e_mat(:,i) = Rot(:,3);
    r_mat(:,i) = r;
    
end

a1 = 0;                         d1 = abs(Mxyz_rpy(3,1));
a2 = abs(Mxyz_rpy(1,3));        d2 = 0;
a3 = abs(Mxyz_rpy(1,4));        d3 = 0;
a4 = 0;                         d4 = abs(Mxyz_rpy(3,4));
a5 = 0;                         d5 = abs(Mxyz_rpy(2,5));
a6 = 0;                         d6 = abs(Mxyz_rpy(2,6));

p_mat = r_mat;

filepath = [robot_type,'_joint_limits_urdf.xml'];
robot_limits = readstruct(filepath);
limits_fields = fieldnames(robot_limits);

q_min = zeros(n_joints,1);
q_max = zeros(n_joints,1);
qp_max = zeros(n_joints,1);
tau_max = zeros(n_joints,1);
for i = 1:n_joints
    
    if strcmp(robot_type,'arm1') || strcmp(robot_type,'arm2')
        qp_max(i) = eval(['robot_limits.', limits_fields{i}, '.max_velocity']);
        qpp_max(i) = eval(['robot_limits.', limits_fields{i}, '.max_acceleration']);
    else
    q_min(i) = eval(['robot_limits.', limits_fields{i}, '.min_position']);
    q_max(i) = eval(['robot_limits.', limits_fields{i}, '.max_position']);
    qp_max(i) = eval(['robot_limits.', limits_fields{i}, '.max_velocity']);
    tau_max(i) = eval(['robot_limits.', limits_fields{i}, '.max_effort']);
    end
end

robot.n_joints = n_joints;
robot.A = A;
robot.a = [a1 a2 a3 a4 a5 a6]; 
robot.d = [d1 d2 d3 d4 d5 d6]; 
robot.alpha = [pi/2 pi/2 pi/2 pi pi/2 pi/2];
robot.e = e_mat;
robot.p = p_mat;
robot.r = r_mat;

if strcmp(robot_type,'arm1') || strcmp(robot_type,'arm2')
    robot.qp_max = qp_max;
    robot.qp_min = -robot.qp_max;
    robot.qpp_max = qpp_max;
    robot.qpp_min = -robot.qpp_max;
else
    robot.q_min = deg2rad(q_min);
    robot.q_max = deg2rad(q_max);
    robot.qp_max = deg2rad(qp_max);
    robot.qp_min = -robot.qp_max;
    robot.tau_max = tau_max;
    robot.tau_min = -robot.tau_max;
end

end