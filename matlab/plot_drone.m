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
    for i = 1:4
        plot3([T.t(1), arms_transformed(1,i)], ... 
              [T.t(2), arms_transformed(2,i)], ...
              [T.t(3), arms_transformed(3,i)], ...
              'Color', color(i), ...
              'LineWidth', line_width);
    end
    
    plot_pose(fig, T, arm_length)
end