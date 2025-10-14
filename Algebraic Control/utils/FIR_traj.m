function [rEx, rEdx, rEddx, rEdddx, rEddddx, rEy, rEdy, rEddy, rEdddy, rEddddy, rEz, rEdz, rEddz, rEdddz, rEddddz, th, thd, thdd, dist, Te, time, n] = FIR_traj(path_type, dim_type, rot, robot, scale)
% Define default values for output variables
    rEx = []; rEdx = []; rEddx = [];
    rEy = []; rEdy = []; rEddy = [];
    rEz = []; rEdz = []; rEddz = [];
    th = []; thd = []; thdd = [];
    dist = 0.0;
    

    dir_name = strcat('FIR_data\',string(scale),'\');
    % s = what; %look in current directory
    % s=what('FIR_data\');
    s = what(dir_name);
    matfiles=s.mat;
    for i=1:numel(matfiles)
        if strcmp(matfiles(i),[path_type,'_',dim_type,'.mat'])
            clear traj;
            traj = load(strcat(dir_name, char(matfiles(i))));
            disp(strcat("loaded ", sprintf("%d",i)))
        end
        % if strcmp(path_type, "ark")
        %     if strcmp(matfiles(i),[path_type,'_x_',dim_type,'.mat'])
        %         trajx = load(char(matfiles(i)));
        %         disp(matfiles(i))
        %     elseif strcmp(matfiles(i),[path_type,'_y_',dim_type,'.mat'])
        %         trajy = load(char(matfiles(i)));
        %         disp(matfiles(i))
        %     end        
        % end
    end
    

    % if ~strcmp(path_type, "ark")
    %     time = traj.out.tout; 
    % else
    %     disp("ark")
    %     time = trajx.out.tout;
    % end
    time = traj.out.tout; 
    Te = time(end);
    n = length(time);

    if rot ~= 0
        [th,thd,thdd] = motion_law(0,rot,0,0,time');
    else
        th   = zeros(1,n);
        thd  = zeros(1,n);
        thdd = zeros(1,n);
    end

    if strcmp(path_type, "ark")
        th   = zeros(1,n);
        thd  = zeros(1,n);
        thdd = zeros(1,n);
    end


    if strcmp(path_type,'x')
        rEx = traj.out.TrajectoryProfiles.signals(1).values';
        rEdx = traj.out.TrajectoryProfiles.signals(2).values';
        rEddx = traj.out.TrajectoryProfiles.signals(3).values';
        rEdddx = traj.out.TrajectoryProfiles.signals(4).values';
        rEddddx = traj.out.TrajectoryProfiles.signals(5).values';

        rEy   = zeros(1,n);
        rEdy  = zeros(1,n);
        rEddy = zeros(1,n);
        rEdddy = zeros(1,n);
        rEddddy = zeros(1,n);

        
        if strcmp(dim_type,'3D')
            % rEz   = (rEy.^2 - rEx.^2)/b^2;
            % rEdz  = (2*rEy.*rEdy - 2*rEx.*rEdx)/b^2;
            % rEddz = (2*rEdy.^2 + 2*rEy.*rEddy  - 2*rEdx.^2 - 2*rEx.*rEddx)/b^2;
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            rEdddz = zeros(1,n);
            rEddddz = zeros(1,n);
        else
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            rEdddz = zeros(1,n);
            rEddddz = zeros(1,n);
            fprintf('Wrong dimension type...')
        end   
        dist = rEx(end);

    elseif strcmp(path_type,'y')
        rEy = traj.out.TrajectoryProfiles.signals(1).values';
        rEdy = traj.out.TrajectoryProfiles.signals(2).values';
        rEddy = traj.out.TrajectoryProfiles.signals(3).values';
        rEdddy = traj.out.TrajectoryProfiles.signals(4).values';
        rEddddy = traj.out.TrajectoryProfiles.signals(5).values';

        rEx   = zeros(1,n);
        rEdx  = zeros(1,n);
        rEddx = zeros(1,n);
        rEdddx  = zeros(1,n);
        rEddddx = zeros(1,n);
        
        if strcmp(dim_type,'3D')
            % rEz   = (rEy.^2 - rEx.^2)/b^2;
            % rEdz  = (2*rEy.*rEdy - 2*rEx.*rEdx)/b^2;
            % rEddz = (2*rEdy.^2 + 2*rEy.*rEddy  - 2*rEdx.^2 - 2*rEx.*rEddx)/b^2;
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            rEdddz = zeros(1,n);
            rEddddz = zeros(1,n);
        else
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            rEdddz = zeros(1,n);
            rEddddz = zeros(1,n);
            fprintf('Wrong dimension type...')
        end   
        dist = rEy(end);

    elseif strcmp(path_type, 'ark')
        rEy = traj.out.Y.signals(1).values';
        rEdy = traj.out.Y.signals(2).values';
        rEddy = traj.out.Y.signals(3).values';
        rEdddy = traj.out.Y.signals(4).values';
        rEddddy = traj.out.Y.signals(5).values';

        rEx = traj.out.X.signals(1).values';
        rEdx = traj.out.X.signals(2).values';
        rEddx = traj.out.X.signals(3).values';
        rEdddx = traj.out.X.signals(4).values';
        rEddddx = traj.out.X.signals(5).values';
        
        if rot == 0
            th   = zeros(1,n);
            thd  = zeros(1,n);
            thdd = zeros(1,n);
        else
            th = -atan2(rEdx,rEdy);
            for i=1:length(rEdx)
                if rEdy(i)< 1e-7 
                    if i>length(rEdx)/2
                        th(i) = pi/2;
                    end
                end
            end

            for i=2:n
                thd(i) = (th(i)-th(i-1))/0.002;
            end
            for i=2:n
                thdd(i) = (thd(i)-thd(i-1))/0.002;
            end
        end

        if strcmp(dim_type,'3D')
            % rEz   = (rEy.^2 - rEx.^2)/b^2;
            % rEdz  = (2*rEy.*rEdy - 2*rEx.*rEdx)/b^2;
            % rEddz = (2*rEdy.^2 + 2*rEy.*rEddy  - 2*rEdx.^2 - 2*rEx.*rEddx)/b^2;
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            rEdddz = zeros(1,n);
            rEddddz = zeros(1,n);
        else
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            rEdddz = zeros(1,n);
            rEddddz = zeros(1,n);
            fprintf('Wrong dimension type...')
        end   
        dist = sqrt(rEx(end)^2+rEy(end)^2);
    elseif strcmp(path_type, 'ark3')
        rEy = traj.out.Y.signals(1).values';
        rEdy = traj.out.Y.signals(2).values';
        rEddy = traj.out.Y.signals(3).values';
        rEdddy = traj.out.Y.signals(4).values';
        rEddddy = traj.out.Y.signals(5).values';

        rEx = traj.out.X.signals(1).values';
        rEdx = traj.out.X.signals(2).values';
        rEddx = traj.out.X.signals(3).values';
        rEdddx = traj.out.X.signals(4).values';
        rEddddx = traj.out.X.signals(5).values';
        
        if rot == 0
            th   = zeros(1,n);
            thd  = zeros(1,n);
            thdd = zeros(1,n);
        else
            th = -atan2(rEdx,rEdy);
            for i=1:length(rEdx)
                if rEdy(i)< 1e-7 
                    if i>length(rEdx)/2
                        th(i) = pi/2;
                    end
                end
            end

            for i=2:n
                thd(i) = (th(i)-th(i-1))/0.002;
            end
            for i=2:n
                thdd(i) = (thd(i)-thd(i-1))/0.002;
            end
        end

        if strcmp(dim_type,'3D')
            rEz = traj.out.Z.signals(1).values';
            rEdz = traj.out.Z.signals(2).values';
            rEddz = traj.out.Z.signals(3).values';
            rEdddz = traj.out.Z.signals(4).values';
            rEddddz = traj.out.Z.signals(5).values';
        elseif strcmp(dim_type,'2D')
            rEz   = zeros(1,n);
            rEdz  = zeros(1,n);
            rEddz = zeros(1,n);
            rEdddz = zeros(1,n);
            rEddddz = zeros(1,n);
        else
            rEz = traj.out.Z.signals(1).values';
            rEdz = traj.out.Z.signals(2).values';
            rEddz = traj.out.Z.signals(3).values';
            rEdddz = traj.out.Z.signals(4).values';
            rEddddz = traj.out.Z.signals(5).values';
            fprintf('Wrong dimension type...')
        end   
        dist = sqrt(rEx(end)^2+rEy(end)^2+rEz(end)^2);

    end

end

