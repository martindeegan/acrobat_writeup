classdef SO3
   properties (Constant)
        dim = 3;
   end

   properties
        matrix = eye(3);
   end
   methods
        % Constructor
        % rot_mat: rotation - 3x3 array. Must have det == 1 and
        % rot_mat*rot_mat' == eye(3)
        function R = SO3(rot_mat)
           assert(size(rot_mat, 1) == SO3.dim);
           assert(size(rot_mat, 2) == SO3.dim);
           assert(abs(det(rot_mat) - 1) <= 1e-6);
           assert(norm(rot_mat * rot_mat' - eye(3)) <= 1e-6);
           R.matrix = rot_mat;
        end

        % Group binary operation or group action on R^3
        function result = mtimes(a, b)
            if isa(b, 'SO3')
                result = SO3(a.matrix * b.matrix);
            else
                result = a.matrix * b; 
            end
            
        end
        
        % Lie group logarithm
        % Maps the group element onto the Lie algebra
        function w = log(R)
            theta = acos((trace(R.matrix) - 1) / 2);
            if theta == 0
                w = [0; 0; 0];
            else
                w = theta/(2 * sin(theta)) * SO3.vee(R.matrix - R.matrix');
            end
        end
        
        % Adjoint operator - linear transformation between a tangent space
        % and the Lie algebra.
        % Returns a 3x3 matrix
        function Ad = adjoint(R)
            Ad = R.matrix;
        end

        % Inverts the rotation
        function R_inv = inverse(R)
            R_inv = SO3(R.matrix');
        end

        % Returns the Euler angles
        % eul = [roll, pitch, yaw]
        function e = eul(R)
            e = flip(rotm2eul(R.matrix))';
        end
   end
   
   methods(Static)
        % Converts the R^3 representation of the Lie algebra into the so3
        % representation of the Lie algebra
        % xi: angular velocity vector - 3x1 array
        function wx = hat(w)
            wx = [0    -w(3)  w(2);
                  w(3)  0    -w(1); 
                 -w(2)  w(1)    0];
        end

        % Converts the so3 representation of the Lie algebra into the R^3
        % representation of the Lie algebra
        % xi_hat: se3 matrix - 3x3 skew-symmetric matrix.
        function w = vee(wx)
            w = [wx(3, 2); wx(1, 3); wx(2,1)];
        end
        
        % Lie algebra exponential map
        % Maps an element of the Lie algebra onto an element of the Lie
        % group
        % xi: angular velocity vector - 3x1 array
        function R = exp(w)
           theta = norm(w);
           if theta == 0
               R = SO3.identity();
           else
               wx = SO3.hat(w / theta);
               R = SO3(eye(3) + wx * sin(theta) + wx^2 * (1 - cos(theta)));
           end
        end
        
        % Returns the identity rotation
        function R = identity()
            R = SO3(eye(3));
        end
        
        function J_r_inv = right_jacobain_inv(Phi)
            phi = norm(Phi);
            a = Phi / phi;
            
            J_r_inv = (phi/2) * cot(phi/2) * eye(3) + (1 - phi/2 * cot(phi/2))*(a*a') + (phi/2)*SO3.hat(a);
        end
   end
end

