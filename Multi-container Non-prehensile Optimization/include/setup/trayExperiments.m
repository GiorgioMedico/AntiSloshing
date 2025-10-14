classdef trayExperiments < robotics.manip.internal.InternalAccess
    properties
        TrayDimensions     % Dimensions of the tray surface [width, length, thickness]
        ContainerHeight    % Height of each container
        ContainerRadius    % Radius of each container
        NumContainers      % Number of containers
        ContainerPositions % Positions of the containers (Nx3 matrix: [x, y, z] for each container)
        ToolOffset % Offset from robot flange to tray
        TrayRotation % Tray rotation wrt 0 configuration of joint 6 
    end

    methods
        function obj = trayExperiments(trayDimensions, trayRotation, containerHeight, containerRadius, numContainers, containerPositions, toolOffset)
            % Constructor to initialize the tray and container properties
            if nargin < 5
                error("All parameters (trayDimensions, containerHeight, containerRadius, numContainers, containerPositions) are required.");
            end

            obj.TrayDimensions = trayDimensions;
            obj.ContainerHeight = containerHeight;
            obj.ContainerRadius = containerRadius;
            obj.NumContainers = numContainers;
            obj.ToolOffset = toolOffset;
            obj.TrayRotation = trayRotation;

            % Validate the container positions size
            if size(containerPositions, 1) ~= numContainers || size(containerPositions, 2) ~= 3
                error("containerPositions must be a Nx3 matrix where N matches numContainers.");
            end
            obj.ContainerPositions = containerPositions;
        end

        function gripper = build(obj)
            % Create a rigid body tree for the tray
            gripper = rigidBodyTree();

            % Create the base of the tray and attachment points
            trayBase = obj.buildTrayBase();
            gripper.addBody(trayBase, gripper.BaseName);

            % Add containers to the tray
            for i = 1:obj.NumContainers
                container = obj.buildContainer(sprintf("container%d", i), obj.ContainerPositions(i, :));
                gripper.addBody(container, trayBase.Name);
            end
        end
    end

    methods (Access = private)
        function trayBase = buildTrayBase(obj)
            % Create tray base
            trayBase = rigidBody("trayBase");
            traySurface = struct('Type', "Box", 'Dim', obj.TrayDimensions, ...
                                 'Tform', trvec2tform([0 0 -obj.TrayDimensions(3)/2]), ...
                                 'Color', [0.7 0.7 0.7]);

            % Rotation matrix for a pi/2 rotation around the Y-axis
            % rotationMatrix = makehgtform('yrotate', pi/2);
            rotationMatrix = makehgtform('yrotate', pi/2)*makehgtform('zrotate', pi);
            rotationMatrix = rotationMatrix*makehgtform('zrotate', obj.TrayRotation);
            
            % Combine the translation and rotation transformations
            traySurface.Tform = traySurface.Tform;

            % Add visual for tray surface
            colordata.rgba = traySurface.Color;
            trayBase.BodyInternal.addVisualInternal(traySurface.Type, traySurface.Dim, traySurface.Tform, colordata);
            trayBase.addCollision(traySurface.Type, traySurface.Dim, traySurface.Tform);

            % Define and attach the fixed joint for the tray
            trayJoint = rigidBodyJoint("trayBaseJoint", "fixed");
            % trayJointTransform = trvec2tform([obj.ToolOffset(3) + obj.TrayDimensions(3) / 2, obj.ToolOffset(2), obj.ToolOffset(1)])*rotationMatrix; % Position the tray slightly above base
            trayJointTransform = trvec2tform([obj.ToolOffset(3) + obj.TrayDimensions(3), obj.ToolOffset(2), obj.ToolOffset(1)])*rotationMatrix; % Position the tray slightly above base
            trayJoint.setFixedTransform(trayJointTransform);
            trayBase.Joint = trayJoint;
        end

        function containerBody = buildContainer(obj, containerName, position)
            % Create a container (cylinder) with the specified position
            containerBody = rigidBody(containerName);
            containerDimensions = [obj.ContainerRadius, obj.ContainerHeight]; % Radius and height for the cylinder

            % Add visual and collision properties for the container
            colordata.rgba = [0.2, 1, 1, 0.5]; % Container color

            z_shift = -(position(3)-obj.ContainerHeight/2);  % Shift cylinder down to align CoM at container frame
            visualTransform = trvec2tform([0, 0, z_shift]);

            containerBody.BodyInternal.addVisualInternal("Cylinder", containerDimensions, visualTransform, colordata);
            containerBody.addCollision("Cylinder", containerDimensions, trvec2tform([0 0 0]));

            % Define and attach the fixed joint for the container
            containerJoint = rigidBodyJoint(sprintf("%s_Joint", containerName), "fixed");
            containerJoint.setFixedTransform(trvec2tform(position)); % Position the container
            containerBody.Joint = containerJoint;
        end
    end
end
