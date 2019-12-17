classdef SO3
   properties (Constant)
        dim = 3;
   end

   properties
        matrix = eye(3);
   end
   methods
        function result = mtimes(a, b)
            if(isa(b, 'SO3'))
                result = SO3(a.matrix * b.matrix);
            else(size(b, 2) == 1)
                result = a.matrix * b; 
            end
            
        end
        
        function R = SO3(rot_mat)
           assert(size(rot_mat, 1) == SO3.dim);
           assert(size(rot_mat, 2) == SO3.dim);
           R.matrix = rot_mat;
        end

        function w = log(R)
            theta = acos((trace(R.matrix) - 1) / 2);
            if theta == 0
                w = [0; 0; 0];
            else
                w = theta * SO3.vee(R.matrix - R.matrix') / (2 * sin(theta));
            end
            
        end

        function Ad = adjoint(R)
            Ad = R.matrix;
        end

        function R_inv = inverse(R)
            R_inv = R;
            R_inv.matrix = R.matrix;
        end
   end
   
   methods(Static)
        function wx = hat(w)
            wx = [0    -w(3)  w(2);
                  w(3)  0    -w(1); 
                 -w(2)  w(1)    0];
        end

        function w = vee(wx)
            w = [wx(3, 2); wx(1, 3); wx(1,2)];
        end

        function R = exp(w)
           theta = norm(w);
           wx = SO3.hat(w / theta);
           R = SO3(eye(3) + wx * sin(theta) + wx^2 * (1 - cos(theta)));
        end
        
        function R = identity()
            R = SO3(eye(3));
        end
   end
end

