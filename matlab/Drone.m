classdef Drone < handle
    properties (Constant)
        g = 9.81;   % Local gravity
        m = 1;      % Mass of the vehicle
        I = eye(3); % Inertia (not used)
        gamma = 0.1; % Drag coeff
    end

    properties
        T = SE3();      % Vehicle pose
        xi = zeros(6,1); % Velocity in se3
        F_net = zeros(6, 1);
    end

    methods
        function D = Drone()
        end

        % Propogates state of drone forward
        % over a time period of dt under
        % a zero order hold approximation
        % with motor force F and angular rates w
        function dynamics_det(D, F_M, w, dt);
            F_g = [0; 0; -D.g * D.m]; % global frame
            F_wind = [0; 0; 0]; % global frame
            F_drag = -D.gamma * D.xi(1:3); % body frame
            F_motor = [0; 0; F_M]; % body frame
            
            % Used to transform vectors in the tangent space
            adj = D.T.adjoint();
            D.F_net = adj(1:3, 1:3) \ (F_g + F_wind) + F_drag + F_motor; % body frame
            
            accel_body = [D.F_net / D.m; zeros(3, 1)];
            
            vel_body = D.xi + accel_body * dt;
            vel_body(4:6) = w;
            
            % Double integrate acceleration
            D.T = D.T * SE3.exp(vel_body * dt + 1/2 * dt^2 + accel_body);
            D.xi = vel_body;
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