close all
pause on
rng(1)

viz = figure('Renderer', 'painters', 'Position', [10 10 900 600])


SIM_STEPS = 200;
STEP_SIZE = 0.01; % Seconds

xi = @(t) 0.25 * [0.1; 0.0; 0.1*(sin(t) + 1); sin(t); 0; pi/20];
T = SE3.identity();

poses = {}
positions = []

t = 0;
for step = 1:SIM_STEPS
    initialize_viz(viz)
    
    if mod(step, 15) == 1
        poses = cat(1, poses, {T});
    end
    
    for i = 1:length(poses)
        plot_pose(viz, poses{i});
    end
    positions = [positions, T.t];
    plot3(positions(1,:), positions(2,:), positions(3,:), 'Color', [1, .75, 0]);
    
    T = T * SE3.exp(xi(t));
    plot_drone(viz, T, 0.5);
    
    t = t + STEP_SIZE;

    finish_viz(viz);
    pause(STEP_SIZE);
end
