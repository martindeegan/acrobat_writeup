classdef SO3
   properties (Constant)
        dim = 3;
   end

   properties
        matrix = eye(3);
   end
   methods
        function R = SO3(rot_mat)
           assert(size(rot_mat, 1) == SO3.dim);
           assert(size(rot_mat, 2) == SO3.dim);
           assert(abs(det(rot_mat) - 1) <= 1e-6);
           assert(norm(rot_mat * rot_mat' - eye(3)) <= 1e-6);
           R.matrix = rot_mat;
        end

        function result = mtimes(a, b)
            if isa(b, 'SO3')
                result = SO3(a.matrix * b.matrix);
            else
                result = a.matrix * b; 
            end
            
        end
        
        function w = log(R)
            theta = acos((trace(R.matrix) - 1) / 2);
            if theta == 0
                w = [0; 0; 0];
            else
                w = theta/(2 * sin(theta)) * SO3.vee(R.matrix - R.matrix');
            end
        end

        function Ad = adjoint(R)
            Ad = R.matrix;
        end

        function R_inv = inverse(R)
            R_inv = SO3(R.matrix');
        end
   end
   
   methods(Static)
        function wx = hat(w)
            wx = [0    -w(3)  w(2);
                  w(3)  0    -w(1); 
                 -w(2)  w(1)    0];
        end

        function w = vee(wx)
            w = [wx(3, 2); wx(1, 3); wx(2,1)];
        end

        function R = exp(w)
           theta = norm(w);
           if theta == 0
               R = SO3.identity();
           else
               wx = SO3.hat(w / theta);
               R = SO3(eye(3) + wx * sin(theta) + wx^2 * (1 - cos(theta)));
           end
        end
        
        function R = identity()
            R = SO3(eye(3));
        end
   end
end

