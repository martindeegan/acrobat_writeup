classdef Drone < handle
    properties (Constant)
        g = 9.81;   % Local gravity
        m = 1;      % Mass of the vehicle
        I = eye(3); % Inertia (not used)
        gamma = 0.1; % Drag coeff
    end

    properties
        T = SE3();      % Vehicle pose
        vel = zeros(6,1); % Body frame velocity in se3
        accel = zeros(6, 1); % Body frame acceleration
        F_net = zeros(6, 1); % Body frame force
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
            F_drag = -D.gamma * D.vel(1:3); % body frame
            F_motor = [0; 0; F_M]; % body frame
            
            % Used to transform vectors in the tangent space
            adj = D.T.adjoint();
            D.F_net = adj(1:3, 1:3) \ (F_g + F_wind) + F_drag + F_motor; % body frame
            D.accel = [D.F_net / D.m; zeros(3, 1)];
            
            % Set angular rate to control input
            D.vel(4:6) = w;
            
            % Double integrate acceleration
            D.T = D.T * SE3.exp(D.vel * dt + 1/2 * dt^2 + D.accel);
            D.vel = D.vel + dt * D.accel;
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