function tray = getTrayParameters(tray_type, len_in, width_in, thickness_in, mu_in)
% getTrayParameters - Gets tray parameters based on type or custom inputs.
%
% Usage:
%   tray = getTrayParameters(tray_type)
%   tray = getTrayParameters(tray_type, len_in, width_in, thickness_in, mu_in)
%
% Description:
%   This function retrieves predefined parameters for common tray types ('plastic', 'wood')
%   or allows for the specification of custom tray dimensions and friction properties.
%   It returns a struct containing all relevant tray attributes.
%
% Required:
%   - tray_type: A string specifying the type of tray.
%                Valid options: 'wood', 'plastic', or 'custom'.
%
% Optional Inputs (required only if tray_type is 'custom'):
%   - len_in       : Length of the tray in meters (scalar).
%   - width_in     : Width of the tray in meters (scalar).
%   - thickness_in : Thickness of the tray in meters (scalar).
%   - mu_in        : Static friction coefficient of the tray surface (scalar, dimensionless).
%
% Output:
%   - tray : A struct containing the following fields:
%            .len       - Tray length (m)
%            .width     - Tray width (m)
%            .thickness - Tray thickness (m)
%            .mu        - Static friction coefficient (dimensionless)
%
% Example:
%   % Get parameters for a plastic tray
%   plastic_tray = getTrayParameters('plastic');
%   disp(plastic_tray);
%
%   % Get parameters for a custom tray
%   custom_tray = getTrayParameters('custom', 0.4, 0.3, 0.02, 0.8);
%   disp(custom_tray);

    % Initialize tray struct
    tray = struct('length', 0, 'width', 0, 'thickness', 0, 'mu', 0);

    switch lower(tray_type)
        case 'plastic'
            tray.length = 0.335;
            tray.width = 0.25;
            % Original thickness was 0.046, updated to 0.052 as per previous script version
            tray.thickness = 0.052; 
            tray.mu = 0.9 * tand(25); % Example friction coefficient calculation
        case 'wood'
            tray.length = 0.36;
            tray.width = 0.36;
            tray.thickness = 0.012;
            tray.mu = 0.9 * tand(25); % Example friction coefficient calculation
        case 'custom'
            % Check if all custom parameters are provided
            if nargin < 5
                error('For ''custom'' tray type, len_in, width_in, thickness_in, and mu_in must be provided.');
            end
            
            % Assign custom values
            tray.length = len_in;
            tray.width = width_in;
            tray.thickness = thickness_in;
            tray.mu = mu_in;
        otherwise
            % Error for unknown tray types
            error('Unknown tray type: ''%s''. Possible types are: ''plastic'', ''wood'', ''custom''.', tray_type);
    end
end
