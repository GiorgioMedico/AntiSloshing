function [rEx, rEdx, rEddx, rEy, rEdy, rEddy, rEz, rEdz, rEddz, qOffset, th, thd, thdd, axisDist, th_max] = SCARA_traj(path_type, dim_type, sigma, sigmad, sigmadd, time, n, Te, T_InFin, robot)
% Define default values for output variables
    rEx = []; rEdx = []; rEddx = [];
    rEy = []; rEdy = []; rEddy = [];
    rEz = []; rEdz = []; rEddz = [];
    qOffset = []; th = []; thd = []; thdd = [];
    axisDist = 0.0;

    if strcmp(path_type,'LE')

        axisDist = 0.0; 
        nb = 5;
        a = 0.5;
        b = 1;
        th_max = nb*pi;
    
        phi   = 2*pi*sigma;
        phid  = 2*pi*sigmad;
        phidd = 2*pi*sigmadd;
    
        [th,thd,thdd] = motion_law(0,th_max,0,0,time);
    
        rEx   = a/2*sin(2*phi+pi);
        rEdx  = a*phid.*cos(2*phi+pi);
        rEddx = a*phidd.*cos(2*phi+pi) - 2*a*phid.^2.*sin(2*phi+pi);
        rEy   = a*cos(phi+pi/2);
        rEdy  = - a*phid.*sin(phi+pi/2);
        rEddy = - a*phidd.*sin(phi+pi/2) - a*phid.^2.*cos(phi+pi/2);
        
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

    elseif strcmp(path_type,'RD')
    
        axisDist = 0.3; 
        nb = 4;
        a = axisDist;
        b = 1.5;
        th_max = nb*pi;
    
        [th,thd,thdd] = motion_law(0,th_max,0,0,time);
    
        rEx   = a*sin(th);
        rEdx  = a*thd.*cos(th);
        rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        rEy   = a*cos(th) - a;
        rEdy  = - a*thd.*sin(th);
        rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th);
    
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
        qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
        % qOffset = [0.4,0.0,-1.9,0.0,-1.8,-0.4+pi];
    
    elseif strcmp(path_type,'FV')
    
        axisDist = 0.3; 
        nb = 2;
        a = axisDist;
        b = 1.5;
        th_max = nb*pi;
    
        % T_InFin = 4;
        T_Cost = Te - T_InFin;
        freq = 500;
        n_Infin = freq*T_InFin + 1;
        n_Cost  = freq*T_Cost +1;
        time_InFin = linspace(0,T_InFin,n_Infin);
        time_Cost  = linspace(0,T_Cost,n_Cost);
    
        [th_InFin,thd_InFin,thdd_InFin] = motion_law(0,th_max,0,0,time_InFin);
    
        thdd = [thdd_InFin(1:(n_Infin-1)/2+1), zeros(1,n_Cost-1), thdd_InFin((n_Infin-1)/2+2:end)];
        thd = cumtrapz(time,thdd);
        th = cumtrapz(time,thd);
    
        rEx   = a*sin(th);
        rEdx  = a*thd.*cos(th);
        rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        rEy   = a*cos(th) - a;
        rEdy  = - a*thd.*sin(th);
        rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th);
    
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
        % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,robot.q_min(6)+pi];
        % qOffset = [0.4,0.0,-1.9,0.0,-1.8,-0.4+pi];
        qOffset = [0.0,-0.1,-1.9,0.0,-1.8,44];
    
    elseif strcmp(path_type,'TRD')
    
        axisDist = 0.3; 
        nb = 1.5;
        a = axisDist;
        b = 1;
        th_max = nb*pi;
    
        [th,thd,thdd] = motion_law(0,th_max,0,0,time);
    
        rEx   = a*sin(th);
        rEdx  = a*thd.*cos(th);
        rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        rEy   = a*cos(th) + b*sigma - a;
        rEdy  = - a*thd.*sin(th) + b*sigmad;
        rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th) + b*sigmadd;
    
        if strcmp(dim_type,'3D')
            rEz   = (a/2)*sin(4/3*th);
            rEdz  = (2*a/3)*thd.*cos(4/3*th);
            rEddz = (2*a/3)*thdd.*cos(4/3*th) - (8*a/9)*thd.^2.*sin(4/3*th);
            qOffset = [0.4,0.0,-1.72,0.0,-1.72,-0.4];
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            qOffset = [0.4,0.0,-1.8,0.0,-1.8,-0.4];
        else
            fprintf('Wrong dimension type...')
        end    
        % qOffset = [0.0,-0.1,-1.9,0.0,-1.8,3.14];
        
    elseif strcmp(path_type,'Line_rot')
        axisDist = 0.0; 
        nb = 1.5;
        a = axisDist;
        b = 1;
        th_max = nb*pi;
    
        [th,thd,thdd] = motion_law(0,th_max,0,0,time);
    
        rEx   = a*sin(th);
        rEdx  = a*thd.*cos(th);
        rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        rEy   = a*cos(th) + b*sigma - a;
        rEdy  = - a*thd.*sin(th) + b*sigmad;
        rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th) + b*sigmadd;
    
        if strcmp(dim_type,'3D')
            rEz   = (a/2)*sin(4/3*th);
            rEdz  = (2*a/3)*thd.*cos(4/3*th);
            rEddz = (2*a/3)*thdd.*cos(4/3*th) - (8*a/9)*thd.^2.*sin(4/3*th);
            qOffset = [0.4,0.0,-1.72,0.0,-1.72,-0.4];
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            qOffset = [0.4,0.0,-1.8,0.0,-1.8,-0.4];
        else
            fprintf('Wrong dimension type...')
        end    

    elseif strcmp(path_type,'Line')
        axisDist = 0.0; 
        nb = 1.5;
        a = axisDist;
        b = 1;
        th_max = nb*pi;
    
        % [th,thd,thdd] = motion_law(0,0,0,0,time);
    
        % rEx   = a*sin(th);
        % rEdx  = a*thd.*cos(th);
        % rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        % rEy   = a*cos(th) + b*sigma - a;
        % rEdy  = - a*thd.*sin(th) + b*sigmad;
        % rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th) + b*sigmadd;
        
        rEx   = zeros(1,n);
        rEdx  = zeros(1,n);
        rEddx = zeros(1,n);
        [rEy, rEdy, rEddy] = septic_poly(0, 0.5, 0, 0, 0, time);

    
        if strcmp(dim_type,'3D')
            [rEz, rEdz, rEddz] = septic_poly(0, 0.3, 0, 0, 0, time);
            qOffset = [0.4,0.0,-1.8,0.0,-1.8,-0.4];
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            qOffset = [0.4,0.0,-1.8,0.0,-1.8,-0.4];
        else
            fprintf('Wrong dimension type...')
        end      

    elseif strcmp(path_type,'Line2D')
        axisDist = 0.0; 
        nb = 1.5;
        a = axisDist;
        b = 1;
        th_max = nb*pi;
    
        % [th,thd,thdd] = motion_law(0,0,0,0,time);
    
        % rEx   = a*sin(th);
        % rEdx  = a*thd.*cos(th);
        % rEddx = a*thdd.*cos(th) - a*thd.^2.*sin(th);
        % rEy   = a*cos(th) + b*sigma - a;
        % rEdy  = - a*thd.*sin(th) + b*sigmad;
        % rEddy = - a*thdd.*sin(th) - a*thd.^2.*cos(th) + b*sigmadd;
        
        rEx   = zeros(1,n);
        rEdx  = zeros(1,n);
        rEddx = zeros(1,n);
        % [rEx, rEdx, rEddx] = septic_poly(0, 0.5, 0, 0, 0, time);

        [rEx(1:ceil(n/2)), rEdx(1:ceil(n/2)), rEddx(1:ceil(n/2))] = septic_poly(0, 0.2, 0, 0, 0, time(1:ceil(n/2)));
        [rEx(ceil(n/2+1):n), rEdx(ceil(n/2+1):n), rEddx(ceil(n/2+1):n)] = septic_poly(0.2, 0, 0, 0, 0, time(ceil(n/2+1):n));
        [rEy, rEdy, rEddy] = septic_poly(0, 0.5, 0, 0, 0, time);

    
        if strcmp(dim_type,'3D')
            [rEz, rEdz, rEddz] = septic_poly(0, 0.5, 0, 0, 0, time);
            qOffset = [0.4,0.0,-1.72,0.0,-1.72,-0.4];
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            qOffset = [0.4,0.0,-1.8,0.0,-1.8,-0.4];
        else
            fprintf('Wrong dimension type...')
        end      
    else
        error('Invalid path type provided.');
    end
end

