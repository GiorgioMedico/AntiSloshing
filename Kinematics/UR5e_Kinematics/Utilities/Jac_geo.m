function J = Jac_geo( q, robot, use_casadi )

n_joints = robot.n_joints;

for i = 1:n_joints

%     joint(i).Y = screwvector(robot.link(i).e,robot.link(i).p,[]);
    Y(:,i) = screwvector(robot.e(:,i),robot.p(:,i),[]);
    
    body(i).A = RotPosToSE3(Rx(robot.alpha(i)),robot.r(:,i));
    if i == 1
        
        body(i).B = body(i).A;
    else
        body(i).B = inv(body(i-1).A)*body(i).A;
    end  
    
%     joint(i).X = Adinv(body(i).A)*joint(i).Y;
    X(:,i) = Adinv(body(i).A)*Y(:,i);
    
end

for i = 1:n_joints
    
    body(i).C = prodexp(Y,q,i)*body(i).A;  
    
end

% Jacobian
if use_casadi 
    Ab = casadi.SX.sym('Ab', 6*n_joints, 6*n_joints);
    Xb = casadi.SX.sym('Xb', 6*n_joints, n_joints);
    Ab = 0*Ab;
    Xb = 0*Xb;
else
    Ab  = zeros(6*n_joints,6*n_joints);
    Asb = zeros(6*n_joints,6*n_joints);
    Xb  = zeros(6*n_joints,n_joints);
end
for i = 1:n_joints
    rows = (6*(i-1)+1):6*i;
    for j = 1:n_joints
        cols = (6*(j-1)+1):6*j;
        if j <= i
            if j == i
                Ab(rows,cols) = eye(6);
%                 Asb(rows,cols) = Ad(body(j).C);
            else
                Cij = RelConf(body(i).C,body(j).C);
                Ab(rows,cols) = Ad(Cij);
%                 Asb(rows,cols) = Ad(body(j).C);
            end
        end
        
        if j == i
            Xb(rows,j) = X(:,j);
        end
    end
    
end

Jb = Ab*Xb;

J = [body(6).C(1:3,1:3) zeros(3,3); zeros(3,3) body(6).C(1:3,1:3)]*Jb(31:36,:);

end