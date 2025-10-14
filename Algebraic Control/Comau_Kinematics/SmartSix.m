classdef SmartSix
    properties
        a
        d
        alpha
        q_min
        q_max
        qp_min
        qp_max
        qpp_min
        qpp_max
        q_c0
        q_dh0
        S
        name
    end

    methods
        % Constructor
        function obj = SmartSix(d6)
            if nargin < 1
                d6 = 0.095;
            end
            obj.a = [0.15, 0.59, 0.13, 0, 0, 0];
            obj.d = [0.45, 0, 0, 0.6471, 0, d6];
            obj.alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];

            obj.q_min = deg2rad([-170 -85 -230 -210 -130 -2700]);
            obj.q_max = deg2rad([170 155 50 210 130 2700]);

            obj.qp_min = [-2.44346, -2.79253, -2.96706, -7.85398, -6.54498, -9.59931];
            obj.qp_max = -obj.qp_min;

            obj.qpp_min = -[6.5, 8.5, 10, 15, 20, 20];
            obj.qpp_max = -obj.qpp_min;

            obj.q_c0 = [0.0; pi/2; -pi/2; 0.0; 0.0; 0.0];
            obj.q_dh0 = [0.0; -pi/2; -pi/2; 0.0; 0.0; 0.0];
            obj.S = diag([-1,1,-1,-1,1,-1]);

            obj.name = "Comau_SmartSix";
        end

        % Forward Kinematics
        function T = fk(obj, q)
            T = SmartSix_FK(obj, q);
        end

        % Inverse Kinematics
        function q = ik(obj, T, sol)
            q = SmartSix_IK(obj, T, sol);
        end

        % Inverse Kinematics
        function q = ik_sym(obj, T, sol)
            q = SmartSix_IK_sym(obj, T, sol);
        end

        % Geometric Jacobian
        function J = Jg(obj,q)
            J = SmartSix_Jg(obj,q);
        end

        % Geometric Jacobian
        function J = Jg_sym(obj,q)
            J = SmartSix_Jg_sym(obj,q);
        end
    end
end
