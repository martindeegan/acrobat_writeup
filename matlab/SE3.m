classdef SE3
   properties (Constant)
        dim = 6;
   end

   properties
        R = SO3.identity()
        t = zeros(3, 1);
   end
   methods
        % Constructor
        % R: rotation - SO3 object or 3x3 array. Must have det == 1 and 
        % R*R' == eye(3)
        % t: translation - 3x1 array
        function T = SE3(R, t)
           if isa(R, 'SO3')
               T.R = R;
           else
               T.R = SO3(R);
           end
           T.t = t;
        end

        % Group binary operation
        % Group action on R^3
        % b - SE3 object or 3xN matrix
        function result = mtimes(a, b)
            if isa(b, 'SE3')
                result = SE3(a.R * b.R, a.t + a.R * b.t);
            else
                result = a.R * b + a.t; 
            end
            
        end
        
        % Lie group logarithm
        % Maps the group element onto the Lie algebra
        function xi = log(T)
            w = T.R.log();

            theta = norm(w);
            if theta == 0
                V = eye(3);
            else
                ux = SO3.hat(w / theta);
                V = eye(3) + (1 - cos(theta))/theta * ux + (theta - sin(theta))/theta * ux^2;
            end
            rho = V \ T.t;

            xi = zeros(SE3.dim, 1);
            xi(1:3) = rho;
            xi(4:6) = w;

        end

        % Adjoint operator - linear transformation between a tangent space
        % and the Lie algebra.
        % Returns a 6x6 matrix
        function Ad = adjoint(T)
            Ad = [T.R.matrix,  SO3.hat(T.t) * T.R.matrix;
                  zeros(3),    T.R.matrix               ];
        end

        % Inverts the pose
        function T_inv = inverse(T)
            T_inv = SE3(T.R.inverse(), -(T.R.inverse() * T.t));
        end
        
        % Returns a 4x4 homogeneous matrix
        function mat = matrix(T)
            mat = eye(4);
            mat(1:3, 1:3) = T.R.matrix;
            mat(1:3, 4) = T.t;
        end
   end
   
   methods(Static)
        % Converts the R^6 representation of the Lie algebra into the se3
        % representation of the Lie algebra
        % xi: twist vector - 6x1 array
        function xi_hat = hat(xi)
            xi_hat = [SO3.hat(xi(4:6)), xi(1:3);
                      zeros(1, 3),      0];
        end

        % Converts the se3 representation of the Lie algebra into the R^3
        % representation of the Lie algebra
        % xi_hat: se3 matrix - 4x4 matrix.
        function xi = vee(xi_hat)
            xi = zeros(6, 1);
            xi(1:3) = xi_hat(1:3, 4);
            xi(4:6) = SO3.vee(xi_hat(1:3, 1:3));
        end

        % Lie algebra exponential map
        % Maps an element of the Lie algebra onto an element of the Lie
        % group
        % xi: twist vector - 6x1 array
        function T = exp(xi)
           rho = xi(1:3);
           w = xi(4:6);           
           theta = norm(w);
           if theta == 0
               V = eye(3)
           else
               ux = SO3.hat(w / theta);
               V = eye(3) + (1 - cos(theta))/theta * ux + (theta - sin(theta))/theta * ux^2
           end
           
           T = SE3(SO3.exp(w), V * rho);
        end
        
        % Returns the identity pose
        function T = identity()
            T = SE3(SO3.identity(), zeros(3, 1));
        end
   end
end

