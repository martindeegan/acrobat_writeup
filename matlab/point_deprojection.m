rng(100)

% =====================================
% Tunable parameters
% =====================================
N_points = 100;
N_poses = 3;

% Pose generation parameters
rotation_scalar = 0.1;
distance_scalar = 25;
rotation_axis = [-1, -0.05, -1];
translation = [0; -2; 0.1];

% Noise on known poses - will be quite high if only using IMU
pose_noise_rot = 0;
pose_noise_trans = 0;

% Noise for image generations
projection_noise = 1;
distortion_noise = 0;
intrinsics_noise = 0;

% =====================================
% Calibrated Camera intrinsics
% =====================================
fx = 278.66723066149086;
fy = 278.48991409740296;
cx = 319.75221200593535;
cy = 241.96858910358173;
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
dist = [-0.013721808247486035, 0.020727425669427896, -0.012786476702685545, 0.0025242267320687625]
% =====================================


% =====================================
% Generate 3D data
% =====================================
% Generate map points
points_gt = [unifrnd(-100, 20, 3, N_points) .* [0.1; 1; 0.5] + [40; 0; 20], unifrnd(-40, 40, 3, N_points) .* [1; 0.01; 0.5] + [0; 20; 10]];

N_points = 2*N_points;

% Generate camera poses

% Velocity
rotation_axis = rotation_axis / norm(rotation_axis);
translation = translation / norm(translation);

% Create initial pose
R = axang2rotm([0, 1, 0, pi/2.2]);
t = [-45; 0; 0];
T_0 = invert_pose([R, t]);
poses_gt = {T_0};

% Integrate velocity
for i = 2:N_poses
    R = R * axang2rotm([rotation_axis, rotation_scalar]);
    t = t + translation * distance_scalar;
    poses_gt = cat(2, poses_gt, {invert_pose([R, t])});
end


% Display 3D data
% figure(1)
% plot_points(points_gt, 'b.');
% hold on
% for i = 1:N_poses
%    pose_cell = poses_gt(i);
%    plot_pose(invert_pose(cell2mat(pose_cell))); 
% end
% hold off
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')


% =====================================
% Generate image data
% =====================================

% figure(2)
% for i = 1:N_poses
%    T = cell2mat(poses_gt(i));
%    projection = proj(T, K, dist, points_gt);
%    subplot(3, 1, i)
%    display_image(projection);
% end

% Estimated points

% Initialize points
points = N_poses * ones(3, N_points);
initial_distance = 1;
for i = 1:1
   T = cell2mat(poses_gt(i));
   projection = proj(T, K, dist, points_gt);
   points(1:2,:) = points(1:2,:) + projection;
end

points = initial_distance * points / N_poses;
points_hom = [points; ones(1, N_points)];
points = invert_pose(T) * points_hom;
points = points(1:3,:);


% Jacobian matrix
A = sparse(2*N_poses*N_points, 3*N_points);

% Create observations
U = zeros(2*N_poses*N_points, 1);
for i = 1:N_poses
   T = cell2mat(poses_gt(i));
   projection = proj(T, K, dist, points_gt);
   range = (i-1)*2*N_points+1:i*2*N_points
   U(range) = reshape(projection, [2*N_points, 1]);
end

max_iterations = 100;
loss_history = zeros(1, max_iterations);
prev_loss = inf;
convergence_thresh = 1e-4;
for itr=1:max_iterations

% Construct jacobian
for i=1:N_poses
    offset = (i-1)*2*N_points;
    for j=1:N_points
        A_ij = dproj(cell2mat(poses_gt(i)), K, dist, points(:,j));
        row_range = ((2*j-1)+2*N_points*(i-1)):((2*j)+2*N_points*(i-1));
        col_range = ((3*j-2):(3*j));
        A(row_range,col_range) = A_ij;
    end
end

P = [];
for i = 1:N_poses
   T = cell2mat(poses_gt(i));
   projection = proj(T, K, dist, points);
   P = [P; reshape(projection, [2*N_points, 1])];
end

err = P - U;
loss = norm(err)
if (prev_loss - loss) < convergence_thresh && (prev_loss - loss) > 0
   break; 
end
prev_loss = loss;
loss_history(itr) = loss;
dp = -(A'*A)\A'*err;
dp = reshape(dp, [3, N_points]);
points = points + dp;
end

display(sprintf('Converged in %i iterations', itr))

figure(5)
plot3(points_gt(1,:), points_gt(2,:), points_gt(3,:), 'b.')
hold on
plot3(points(1,:), points(2,:), points(3,:), 'go')
hold off
axis equal

% figure(6)
% plot(loss_history);
% figure(7)
% spy(A)

function result = d(i, theta)
    theta_stacked = [theta.^3; theta.^5; theta.^7; theta.^9];
    result = theta + i*theta_stacked;
end

function proj_points = proj(T,K,i,p)
    N = size(p);
    N_points = N(2);
    p_hom = [p; ones(1, N_points)];
    pts = T * p_hom;
    
    xy = pts(1:2, :);
    z = pts(3,:);
    r = vecnorm(xy);
    theta = atan2(r, z);
    dtheta = d(i, theta);
    
    proj_points = K(1:2,1:2)*((dtheta ./ r) .* xy) + K(1:2,3);
end

function dx = dxdp()
    dx = [1, 0, 0];
end

function dy = dydp()
    dy = [0, 1, 0];
end

function dz = dzdp()
    dz = [0, 0, 1];
end

function dr = drdp(p)
    x = p(1);
    y = p(2);
    dr = 0.5 / sqrt(x^2 + y^2) * [2*x, 2*y, 0];
end

function dtheta = dthetadp(p, r)
    z = p(3);

    dtheta = 1 / (1 + (r/z)^2) * (drdp(p)/z - r/(z^2)*dzdp());
end

function dd = dddp(dist, theta, p, r)
    dddtheta = (1 + 3*dist(1)*theta.^2 + 5*dist(2)*theta.^4 + 7*dist(3)*theta.^6 + 9*dist(4)*theta.^8);
    dd = dddtheta * dthetadp(p, r);
end

function A = dproj(T, K, dist, p)
    N = size(p);
    p_hom = [p; ones(1, N(2))];
    p_ = T*p_hom;
    R = T(1:3,1:3);
    
    r = norm(p_(1:2));
    x = p_(1);
    y = p_(2);
    z = p_(3);
    theta = atan2(r,z);
    dtheta = d(dist, theta);
    
    f_x = K(1,1);
    f_y = K(2,2);
    dudp = f_x * (dddp(dist, theta, p_, r) * x / r + dtheta * (dxdp()/r - x/(r^2)*drdp(p_)));
    dvdp = f_y * (dddp(dist, theta, p_, r) * y / r + dtheta * (dydp()/r - y/(r^2)*drdp(p_)));
    
    A = [dudp; dvdp] * R;
end

function T_inv = invert_pose(T)
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    T_inv = [R', -R'*t];
end

function plot_points(points, style)
    plot3(points(1,:), points(2,:), points(3,:), style)
end

function plot_pose(T)
    basis = eye(3) * 3;
    R = T(1:3, 1:3);
    t = T(1:3,4);
    transformed_basis = R * basis + t;
    plot3([t(1), transformed_basis(1, 1)], [t(2), transformed_basis(2, 1)], [t(3), transformed_basis(3, 1)], 'r') 
    plot3([t(1), transformed_basis(1, 2)], [t(2), transformed_basis(2, 2)], [t(3), transformed_basis(3, 2)], 'g') 
    plot3([t(1), transformed_basis(1, 3)], [t(2), transformed_basis(2, 3)], [t(3), transformed_basis(3, 3)], 'b') 
end

function display_image(projected_points)
    plot(projected_points(1,:), projected_points(2,:), 'r.');
    axis equal
    xlim([0, 640]);
    ylim([0, 480]);
end