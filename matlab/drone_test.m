clear; close all; format compact;

tmin = 0;
tmax = 10;
dt = 0.001;
t = linspace(tmin,tmax,(tmax-tmin)/dt + 1);

F = [0;0;9.81];
w = [0;0;0];
drone = Drone;

x_sol = zeros(length(t), 6);
for i = 2:length(t)
    drone.dynamics_det(F, w, dt)

    x_sol(i,:) = drone.getState();
end

figure;
subplot(311)
plot(t, x_sol(:,1));
subplot(312)
plot(t, x_sol(:,2));
subplot(313)
plot(t, x_sol(:,3));

figure;
subplot(311)
plot(t, x_sol(:,4));
subplot(312)
plot(t, x_sol(:,5));
subplot(313)
plot(t, x_sol(:,6));