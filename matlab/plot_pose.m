function plot_pose(fig, T, length)
    if nargin == 3
        arm_length = length;
    else
        arm_length = 0.15;
    end
    % Basis vectors
    basis = arm_length * eye(3);
    basis = T * basis;
    
    color = ['r', 'g', 'b'];
    line_width = 1.4;
    for i = 1:3
        plot3([T.t(1), basis(1,i)], [T.t(2), basis(2,i)], [T.t(3), basis(3,i)], 'Color', color(i), 'LineWidth', line_width);
    end
end