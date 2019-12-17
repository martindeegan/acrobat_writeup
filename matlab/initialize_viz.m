function initialize_viz(fig)
    plot3(0, 0, 0);
    hold on
    axis equal
    axis vis3d
    grid on
    
    % Limits in meters
    X_LIM = 5;
    Y_LIM = 5;
    Z_LIM = 5;
    GROUND_Z = 1;
    
    xlim([-X_LIM, X_LIM]);
    ylim([-Y_LIM, Y_LIM]);
    zlim([-GROUND_Z, Z_LIM]);
    
    GRID_SIZE = 10;
    % Plot grid
    color = 0.7;
    line_width = 0.25;
    
    [x y] = meshgrid(-GRID_SIZE:1:GRID_SIZE); % Generate x and y data
    z = zeros(size(x, 1)); % Generate z data
    surf(x, y, z, 'FaceAlpha', 0.5) % Plot the surface
    
end