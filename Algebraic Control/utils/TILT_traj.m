function [rEx, rEdx, rEddx, rEy, rEdy, rEddy, rEz, rEdz, rEddz, qOffset, psi, psid, psidd, axisDist, A_psi] = TILT_traj(path_type, dim_type, sigma, sigmad, sigmadd, time, n, dx, dy, dz, Te, robot)
% Define default values for output variables
    rEx = []; rEdx = []; rEddx = [];
    rEy = []; rEdy = []; rEddy = [];
    rEz = []; rEdz = []; rEddz = [];
    qOffset = []; psi = []; psid = []; psidd = [];
    axisDist = 0.0;
    coR = [dx;dy;dz];

    if strcmp(path_type,'Tilt_LE')

        axisDist = 0.0; 
        nb = 2;
        a = 0.5;
        b = 1;
        A_psi = deg2rad(30);
        th_max = nb*pi;
    
        [th,thd,thdd] = motion_law(0,th_max,0,0,time);
    
        rEx   = a/2*sin(2*th+pi);
        rEdx  = a*thd.*cos(2*th+pi);
        rEddx = a*thdd.*cos(2*th+pi) - 2*a*thd.^2.*sin(2*th+pi);
        rEy   = a*cos(th+pi/2);
        rEdy  = - a*thd.*sin(th+pi/2);
        rEddy = - a*thdd.*sin(th+pi/2) - a*thd.^2.*cos(th+pi/2);
    
        psi   = A_psi*sin(2*th);
        psid  = 2*A_psi*thd.*cos(2*th);
        psidd = 2*A_psi*thdd.*cos(2*th) - 4*A_psi*thd.^2.*sin(2*th);
        
        if strcmp(dim_type,'3D')
            rEz   = (rEy.^2 - rEx.^2)/b^2;
            rEdz  = (2*rEy.*rEdy - 2*rEx.*rEdx)/b^2;
            rEddz = (2*rEdy.^2 + 2*rEy.*rEddy  - 2*rEdx.^2 - 2*rEx.*rEddx)/b^2;
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
        else
            fprintf('Wrong dimension type...')
        end    
        qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
    
    elseif strcmp(path_type,'Tilt_RD')
    
        axisDist = 0.3; 
        nb = 4;
        a = axisDist;
        b = 1.5;
        A_psi = deg2rad(30);
        th_max = nb*pi;
    
        [th,thd,thdd] = motion_law(0,th_max,0,0,time);
    
        rEx   = a*sin(th);
        rEdx  = a*thd.*cos(th);
        rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        rEy   = a*cos(th) - a;
        rEdy  = - a*thd.*sin(th);
        rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th);
    
        psi   = A_psi*sin(th);
        psid  = A_psi*thd.*cos(th);
        psidd = A_psi*thdd.*cos(th) - A_psi*thd.^2.*sin(th);
    
        if strcmp(dim_type,'3D')
            rEz   = - (rEx.^2 - (rEy + a).^2)/b^2 - a^2/b^2;
            rEdz  = - (2*rEx.*rEdx - 2*(rEy + a).*rEdy)/b^2;
            rEddz = - (2*rEdx.^2 + 2*rEx.*rEddx  - 2*rEdy.^2 - 2*(rEy + a).*rEddy)/b^2;
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n); 
            rEddz = zeros(1,n);
        else
            fprintf('Wrong dimension type...')
        end    
        % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
        % qOffset = [0.4,0,-1.8,0,-1.8,-0.4];
        % qOffset = [0.0000   -0.0994   -1.8237    0.0000   -1.7243    3.1400];
        qOffset = [0.0000   -0.0790   -1.6522    0.0000   -1.5732    3.1400];
    
    elseif strcmp(path_type,'Tilt_TRD')
    
        axisDist = 0.3; 
        nb = 1.5;
        a = axisDist;
        b = 1;
        A_psi = deg2rad(30);
        th_max = nb*pi;
    
        [th,thd,thdd] = motion_law(0,th_max,0,0,time);
    
        rEx   = a*sin(th);
        rEdx  = a*thd.*cos(th);
        rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        rEy   = a*cos(th) + b*sigma - a;
        rEdy  = - a*thd.*sin(th) + b*sigmad;
        rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th) + b*sigmadd;
    
        psi   = A_psi*sin(4/3*th);
        psid  = 4/3*A_psi*thd.*cos(4/3*th);
        psidd = 4/3*A_psi*thdd.*cos(4/3*th) - 16/9*A_psi*thd.^2.*sin(4/3*th);
    
        if strcmp(dim_type,'3D')
            rEz   = (a/2)*sin(4/3*th);
            rEdz  = (2*a/3)*thd.*cos(4/3*th);
            rEddz = (2*a/3)*thdd.*cos(4/3*th) - (8*a/9)*thd.^2.*sin(4/3*th);
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
        else
            fprintf('Wrong dimension type...')
        end    
        % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
        % qOffset = [0.6,0.0,-1.8,0.0,-1.8,3.14];
        qOffset = [0.4,0,-1.8,0,-1.8,-0.4];
    
    elseif strcmp(path_type,'Tilt')
        A_psi = deg2rad(30);
    
        nb = 1.5;
        b = 1;
    
        psi = zeros(n,1)';
        psid = zeros(n,1)';
        psidd = zeros(n,1)';
        
        % angle trajectory
        [psi(1:ceil(n/2)), psid(1:ceil(n/2)), psidd(1:ceil(n/2))] = quintic_poly(0, A_psi, 0, 0, time(1:ceil(n/2)));
        [psi(ceil(n/2+1):n), psid(ceil(n/2+1):n), psidd(ceil(n/2+1):n)] = quintic_poly(A_psi, 0, 0, 0, time(ceil(n/2+1):n));
    
        % pos, vel, acc of cor in 0 reference frame
        rEx   = zeros(1,n);
        rEdx  = zeros(1,n);
        rEddx = zeros(1,n);
        rEy   = zeros(1,n);
        rEdy  = zeros(1,n);
        rEddy = zeros(1,n);
    
    
        if strcmp(dim_type,'3D')
            [rEz, rEdz, rEddz] = quintic_poly(0, 0.3, 0, 0, time);
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
        else
            fprintf('Wrong dimension type...')
        end    
        % qOffset = [0.4,0,-1.8,0,-1.8,-0.4];
        qOffset = [0.0,0,-1.8,0,-1.8,-pi/4];
    
        % psi = atan(rEddy ./ (g + rEddz)); % Use rEddz = 0 for 2D case
        % psid = gradient(psi, time); % Calculate derivative of psi
        % psidd = gradient(psid, time); % Calculate second derivative of psi
    
        % psi = zeros(1,n);
        % psid = zeros(1,n);
        % psidd = zeros(1,n);
    
        % 
        % dz = (h/2+hn); 
        % dy = -0.2;
        % dz = 0; 
        % dy = 0.2;
        % coR = [0;dy;dz];
    
        % dx = coR(1); % Displacement along x
        % dy = coR(2); % Displacement along y
        % dz = coR(3); % Displacement along z
        
        % Update rEy based on the rotation about the displaced point
        % rEy_act   = rEy + dz * (1 - cos(psi)) - dy * sin(psi);
    
    
        for i=1:n
            % rEy_act(:,i)   = rEy(:,i) + dz * (1 - cos(psi(:,i))) - dy * sin(psi(:,i));
            % v = -Rx(-psi(i))*coR +coR;
            % rEy_act2(:,i) = rEy(:,i) + v(2); 
            v = Rx(-psi(i))*coR;
            v_dot = skew([psid(:,i);0;0])*Rx(-psi(i))*coR;
            v_ddot = skew([psidd(:,i);0;0])*Rx(-psi(i))*coR;
            v_dot2 =  skew([psid(:,i);0;0])*(skew([psid(:,i);0;0])*Rx(-psi(i))*coR);
    
            rEy_act(:,i) = rEy(:,i) - v(2);
            rEdy_act(:,i) = rEdy(:,i) - v_dot(2);
            rEddy_act(:,i) = rEddy(:,i) - v_ddot(2) - v_dot2(2);
    
            rEz_act(:,i) = rEz(:,i) - v(3);
            rEdz_act(:,i) = rEdz(:,i) - v_dot(3);
            rEddz_act(:,i) = rEddz(:,i) - v_ddot(3) - v_dot2(3);
        end
        % rEdy_act  = rEdy + dz * sin(psi) .* psid - dy * cos(psi) .* psid; % Derivative with respect to time
        % rEddy_act = rEddy + dz * (cos(psi) .* psid.^2 - sin(psi) .* psidd) ...
        %               - dy * (sin(psi) .* psid.^2 + cos(psi) .* psidd); % Second derivative
    elseif strcmp(path_type,'Tilt_y')
        A_psi = deg2rad(30);
        % A_psi = 0;
        
        nb = 1.5;
        b = 1;
    
        psi = zeros(n,1)';
        psid = zeros(n,1)';
        psidd = zeros(n,1)';
    
        [psi(1:ceil(n/2)), psid(1:ceil(n/2)), psidd(1:ceil(n/2))] = quintic_poly(0, A_psi, 0, 0, time(1:ceil(n/2)));
        [psi(ceil(n/2+1):n), psid(ceil(n/2+1):n), psidd(ceil(n/2+1):n)] = quintic_poly(A_psi, 0, 0, 0, time(ceil(n/2+1):n));
    
        % psi = zeros(n,1)';
        % psid = zeros(n,1)';
        % psidd = zeros(n,1)';
    
        rEx   = zeros(1,n);
        rEdx  = zeros(1,n);
        rEddx = zeros(1,n);
        % rEy   = zeros(1,n);
        % rEdy  = zeros(1,n);
        % rEddy = zeros(1,n);
    
    
        [rEy, rEdy, rEddy] = quintic_poly(0, 0.5, 0, 0, time);
    
    
    
        if strcmp(dim_type,'3D')
            % rEz   = (a/2)*sin(4/3*th);
            % rEdz  = (2*a/3)*thd.*cos(4/3*th);
            % rEddz = (2*a/3)*thdd.*cos(4/3*th) - (8*a/9)*thd.^2.*sin(4/3*th);
            [rEz, rEdz, rEddz] = quintic_poly(0, 0.3, 0, 0, time);
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
        else
            fprintf('Wrong dimension type...')
        end    
        % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
        % qOffset = [0.6,0.0,-1.8,0.0,-1.8,3.14];
        % qOffset = [0.4,0,-1.8,0,-1.8,-0.4];
        qOffset = [0.0,0,-1.8,0,-1.8,-pi/4];
    
        % psi = atan(rEddy ./ (g + rEddz)); % Use rEddz = 0 for 2D case
        % psid = gradient(psi, time); % Calculate derivative of psi
        % psidd = gradient(psid, time); % Calculate second derivative of psi
    
        % psi = zeros(1,n);
        % psid = zeros(1,n);
        % psidd = zeros(1,n);
        % dz = (h/2+hn); 
        % dz = 0; 
        % dy = 0;
        % coR = [0;dy;dz];
        % coR = [0;0.0;0];
    
        % dx = coR(1); % Displacement along x
        % dy = coR(2); % Displacement along y
        % dz = coR(3); % Displacement along z
        
        % Update rEy based on the rotation about the displaced point
        % rEy_act   = rEy + dz * (1 - cos(psi)) - dy * sin(psi);
    
    
        for i=1:n
            % rEy_act(:,i)   = rEy(:,i) + dz * (1 - cos(psi(:,i))) - dy * sin(psi(:,i));
            % v = -Rx(-psi(i))*coR +coR;
            % rEy_act2(:,i) = rEy(:,i) + v(2); 
            v = Rx(-psi(i))*coR;
            v_dot = skew([psid(:,i);0;0])*Rx(-psi(i))*coR;
            v_ddot = skew([psidd(:,i);0;0])*Rx(-psi(i))*coR;
            v_dot2 =  skew([psid(:,i);0;0])*(skew([psid(:,i);0;0])*Rx(-psi(i))*coR);
    
            rEy_act(:,i) = rEy(:,i) - v(2);
            rEdy_act(:,i) = rEdy(:,i) - v_dot(2);
            rEddy_act(:,i) = rEddy(:,i) - v_ddot(2) - v_dot2(2);
    
            rEz_act(:,i) = rEz(:,i) - v(3);
            rEdz_act(:,i) = rEdz(:,i) - v_dot(3);
            rEddz_act(:,i) = rEddz(:,i) - v_ddot(3) - v_dot2(3);
        end
        % rEdy_act  = rEdy + dz * sin(psi) .* psid - dy * cos(psi) .* psid; % Derivative with respect to time
        % rEddy_act = rEddy + dz * (cos(psi) .* psid.^2 - sin(psi) .* psidd) ...
        %               - dy * (sin(psi) .* psid.^2 + cos(psi) .* psidd); % Second derivative
    end
end

