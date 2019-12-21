
R_ref = SO3.exp(5*randn(3, 1));
ref_log = R_ref.inverse().log();
J_r_inv = SO3.right_jacobain_inv(ref_log);

r = 0.5;
F_des = SO3.exp(r * ones(3, 1) / norm(ones(3, 1))) * ref_log;
F_vec = F_des / norm(F_des);

f_0_lin = @(x) (1/2)*norm(ref_log + J_r_inv*x)^2;

err = [];
thetas = [-pi:0.01:pi];
err_min = 100;
for theta = thetas
   angle_axis = theta * F_vec;
   R_theta = SO3.exp(angle_axis);
      
   rot_err = R_theta.inverse() * R_ref;
   err = [err; [(1/2)*norm(rot_err.log())^2,  f_0_lin(angle_axis)]];
   if (1/2)*norm(rot_err.log())^2 < err_min
      err_min = (1/2)*norm(rot_err.log())^2;
      theta_min = theta;
   end
end

% Solve for optimal theta
w = ref_log;
theta_star = -(w'*J_r_inv*F_vec)/(F_vec'*(J_r_inv'*J_r_inv)*F_vec); 

angle_axis = theta_star * F_vec;
R_theta = SO3.exp(angle_axis);
rot_err = R_theta.inverse() * R_ref;
p_star_approx = (1/2)*norm(rot_err.log())^2;

figure(1)
plot(thetas, err(:, 1));
xlabel('\theta');
ylabel('objective value');
saveas(gcf, 'objective_function.png');

figure(2)
plot(thetas, err)
hold on
scatter(theta_star, p_star_approx, 'go');
hold off
legend({'Error rads', 'Linearized error'});
xlabel('\theta');
ylabel('objective value');
axis equal
title('Solution')
ylim([0, 2*pi]);
saveas(gcf, 'linearized_solution.png');

% function y = huber(x, delta)
%     if abs(x) < delta
%         y = 0.5*x^2;
%     else
%         y = delta*(abs(x) - 0.5*delta);
%     end
% end