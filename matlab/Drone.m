classdef Drone < handle
    properties (Constant)
        g = 9.81;   % Local gravity
        m = 1;      % Mass of the vehicle
        I = eye(3); % Inertia (not used)
    end

    properties
        T = SE3();      % Vehicle pose
        v = zeros(6,1); % Velocity in se3
    end

    methods
        function D = Drone()
        end

        % Propogates state of drone forward
        % over a time period of dt under
        % a zero order hold approximation
        % with force F and angular rates w
        function dynamics_det(D, F, w, dt);

            a = (D.T.R*F + [0;0;-D.g]) / D.m;

            % Propagate velocity forward
            % and set the commanded angular
            % rate
            D.v(1:3) = D.v(1:3) + a * dt;
            D.v(4:6) = w;

            % Propagate pose forward
            D.T = D.T * SE3.exp(D.v * dt);
        end

        % Adds perturbations to force 
        % and rate inputs
        function dynamics_stoc(D,F,w,dt)
            sig_f = 1;
            sig_w = 0.1;

            F = F + [normrnd(0,sig_f);normrnd(0,sig_f);normrnd(0,sig_f)];
            w = w + [normrnd(0,sig_w);normrnd(0,sig_w);normrnd(0,sig_w)];

            D.dynamics_det(F,w,dt);
        end

        function x = getState(D)
            x = [D.T.t', D.T.R.eul'];
        end
    end

end