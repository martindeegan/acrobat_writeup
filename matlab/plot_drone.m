function plot_drone(fig, T, length)
    if nargin == 3
        arm_length = length;
    else
        arm_length = 0.25;
    end
    
    
    arms = arm_length * [
            0.707 -0.707 -0.707  0.707;
            0.707  0.707 -0.707 -0.707;
            0      0      0      0    ];
        
    arms_transformed = T * arms;
    
    color = ['r', 'g', 'g', 'r'];
    line_width = 2.5;
    motor_size = 15.0;
    for i = 1:4
        % Plot arm
        plot3([T.t(1), arms_transformed(1,i)], ... 
              [T.t(2), arms_transformed(2,i)], ...
              [T.t(3), arms_transformed(3,i)], ...
              'Color', color(i), ...
              'LineWidth', line_width);
          
        % Plot motor ball
        plot3(arms_transformed(1,i), ...
              arms_transformed(2,i), ...
              arms_transformed(3,i), ...
              '.', ...
              'Color', color(i), ...
              'MarkerSize', motor_size);

    end
    
    plot_pose(fig, T, arm_length)
end