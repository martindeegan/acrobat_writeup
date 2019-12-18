close all
pause on
rng(1)

viz = figure('Renderer', 'painters', 'Position', [10 10 900 600])


SIM_STEPS = 200;
STEP_SIZE = 0.001; % Seconds
SIM_WAIT = true; % Wait in loop to have a consistent framerate
SIM_RENDER = true; % Enable/Disable rendering to speedup

xi = @(t) 0.25 * [0.0; 0.0; 0.15; 0.2 * sin(t); 0.2 * cos(t); pi/15];
T = SE3.identity();

poses = {};
positions = [];
times = []

D = Drone;

t = 0;
for step = 1:SIM_STEPS
    tic
        
    % ====================
    % Simulate
    % ====================
    
    % Oscillate up and down
    F_M = 9.8 + 0.1 * sin(step * 0.1) + 1/(2*step);
    
    ang_vel_mag = 0.2;
    w = ang_vel_mag * randn(3, 1);
%     w = [ang_vel_mag * sin(step * 0.1); ang_vel_mag * cos(step * 0.1); 0];
    D.dynamics_det(F_M, w, STEP_SIZE);
        
    % ====================
    % Record data
    % ====================
    
    % Save the pose every 15 steps
    if mod(step, 30) == 1
        poses = cat(1, poses, {D.T});
    end
    
    % Save positions
    positions = [positions, D.T.t];

    
    % ====================
    % Do visualizations
    % ====================
    
    if SIM_RENDER
        initialize_viz(viz)

        plot_drone(viz, D, 0.5);

        for i = 1:length(poses)
            plot_pose(viz, poses{i}, 0.3);
        end

        plot3(positions(1,:), positions(2,:), positions(3,:), 'Color', [1, .75, 0]);
        
        finish_viz(viz);
    end

    % Increase sim time
    times = [times, t];
    t = t + STEP_SIZE;
    
    elapsed = toc;
    if elapsed < STEP_SIZE && SIM_WAIT && SIM_RENDER
        pause(STEP_SIZE - elapsed);
    end
end

% ====================
% Make plots
% ====================

% Do final render of visualizer
initialize_viz(viz)

plot_drone(viz, D, 0.5);
for i = 1:length(poses)
    plot_pose(viz, poses{i}, 0.3);
end
plot3(positions(1,:), positions(2,:), positions(3,:), 'Color', [1, .75, 0]);

finish_viz(viz);

% Render orthographic view
figure(2)
subplot(2, 2, 1)
title('Orthographic View')
plot(positions(1,:), positions(2,:));
axis equal
title('XY Positions');
xlabel('X (m)');
ylabel('Y (m)');

subplot(2, 2, 2)
plot(positions(2,:), positions(3,:));
axis equal
title('YZ Positions');
xlabel('Y (m)');
ylabel('Z (m)');

subplot(2, 2, 3)
plot(positions(1,:), positions(3,:));
axis equal
title('XZ Positions');
xlabel('X (m)');
ylabel('Z (m)');

% Plot position timeseries
figure(3)
subplot(3, 1, 1)
plot(times, positions(1,:));
xlabel('time (s)');
ylabel('X (m)');

subplot(3, 1, 2)
plot(times, positions(2,:));
xlabel('time (s)');
ylabel('Y (m)');

subplot(3, 1, 3)
plot(times, positions(3,:));
xlabel('time (s)');
ylabel('Z (m)');
