% Copyright (C) 2019 Maitreya Venkataswamy - All Rights Reserved

%% Setup

clc;
clear;
close all;
rng(0);

%% Cart-pendulum problem definition

% Initial state
x_0 = [0.0; 0.0];

% Target state
x_star = [pi; 0.0];

% Time horizon
t_f = 5.0;

% Number of states along trajectory
N = floor(t_f ./ 0.01);

% Maximum magnitude of control
u_max = [10.0];

% Initialize dynamics
fprintf("initializing inverted pendulum dynamics...\n")
m = 1.0;
l = 1.0;
b = 1.0;
dyn = InvertedPendulumDynamics(m, l, b);

% Initialize cost
fprintf("initializing quadratic cost function...\n")
Q_f = [100.0 0.0;
       0.0 100.0];
R = 0.0001;
cost = QuadraticCost(Q_f, R);

% Number of DDP iterations
num_iter = 50;

% DDP line search parameter
alpha = 0.5;

% Video framerate
fps = 10;

%% Execution of DDP

fprintf("executing DDP...");

tic;
sol = ddp(x_0, x_star, t_f, N, dyn, cost, u_max, num_iter, alpha);
toc;

%% Begin post-processing of solution

if sol.error == 1
    fprintf("Error\n");
end

% Extract the pendulum angle information from the solution
theta = zeros(1, length(sol.x));
theta_dot = zeros(1, length(sol.x));
u = zeros(1, length(sol.x));
for k = 1:N
    theta(k) = sol.x{k}(1);
    theta_dot(k) = sol.x{k}(2);
    u(k) = sol.u{k};
end

%% Create video of system

% Turn off plot visibility temporarily during frame rendering
set(0,'DefaultFigureVisible','off');

% Create figure for frames
fig = figure;
ax = gca;
ax.NextPlot = 'ReplaceChildren';

% Compute number of frames
num_frame = length(1:ceil((1.0 ./ fps) ./ sol.dt):N);

% Structure to store the frames
frames(num_frame) = struct('cdata',[],'colormap',[]);

% Plot limits
xl = [];
yl = [];

% Counter for frames processed
n = 1;

% Open video for writing
%vid = VideoWriter('inverted-pendulum', 'Uncompressed AVI');
%vid.FrameRate = fps;
%open(vid);

% Loop over points in time to process frames at
for k = 1:ceil((1.0 ./ fps) ./ sol.dt):N
    % Plot the pendlum root
    plot(0.0, 0.0, 'ro');
    
    % Apply plot hold
    hold on;
    
    % Coordinated of the tip of the pole
    tipz = l .* sin(theta(k));
    tipy = -l .* cos(theta(k));
    
    % Plot the pendulum
    plot([0.0, tipz], [0.0, tipy], '-b');
    
    % Remove plot hold
    hold off;

    % Obtain or apply plot limits for consistency across frames
    if k == 1
        ylim([-2 .* l, 2 .* l]);
        axis equal;
        xl = xlim();
        yl = ylim();
    else
        xlim(xl);
        ylim(yl);
    end
    
     % Apply blank background
    axis off;

    % Render the frame
    drawnow;
    
    % Record the frame and store in the structure
    frames(n) = getframe(fig);
    
    % Write frame to video
    %writeVideo(vid, frames(n));
    
    % Display progress to command line
    fprintf("rendering video frame %d out of %d...\n", n, num_frame);
    
    % Increment frame counter
    n = n + 1;
end

% Renable plot visibility to view movie and other plots
set(0,'DefaultFigureVisible','on')

% Create fresh figure object for the movie
figure;

% Render the movie and set to loop for a very large number of times
fprintf("done rendering video frames, presenting video...\n")
movie(gcf, frames, 9999, fps);

%% Plot trajectories

% Plot angle history
figure;
pbaspect([5 3 1])
hold on;
plot(sol.t, rad2deg(theta), "k", "LineWidth", 2);
grid on;
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Pendulum Angle [deg]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/theta.png", "-dpng", "-r500")

% Plot anglular velocity history
figure;
pbaspect([5 3 1])
hold on;
plot(sol.t, rad2deg(theta_dot), "k", "LineWidth", 2);
grid on;
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Pendulum Anglular Velocity [deg/s]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/theta_dot.png", "-dpng", "-r500")

% Plot trajectory in state space
figure;
pbaspect([5 3 1])
hold on;
plot(rad2deg(theta), rad2deg(theta_dot), "k", "LineWidth", 2);
plot(rad2deg(x_0(1)), rad2deg(x_0(2)), "o", "MarkerFaceColor", "blue", ...
                          "MarkerEdgeColor", "blue");
plot(rad2deg(x_star(1)), rad2deg(x_star(2)), "o", "MarkerFaceColor", ...
                                     "green", "MarkerEdgeColor", "green");
grid on;
xlabel("Pendulum Angle [deg]", "Interpreter", "latex", "FontSize", 20);
ylabel("Pendulum Angular Speed [deg/s]", "Interpreter", "latex", ...
       "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/traj.png", "-dpng", "-r500")

% Plot control sequence
figure;
pbaspect([5 3 1])
hold on;
plot(sol.t, u, "b", "LineWidth", 2);
grid on;
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Control Input [N]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/u.png", "-dpng", "-r500")

% Plot cost function vs iteration
figure;
pbaspect([5 3 1])
hold on;
plot(1:length(sol.J), sol.J, "r", "LineWidth", 2);
grid on;
xlabel("DDP Iteration [-]", "Interpreter", "latex", "FontSize", 20);
ylabel("Cost Function [-]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
ax.YScale = 'log';
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/J.png", "-dpng", "-r500")

% Plot control energy vs iteration
figure;
pbaspect([5 3 1])
hold on;
plot(1:length(sol.E), sol.E, "r", "LineWidth", 2);
grid on;
xlabel("DDP Iteration [-]", "Interpreter", "latex", "FontSize", 20);
ylabel("Control Energy Usage [$\rm{N}^{2}\rm{m}^{2}\rm{s}$]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
ax.YScale = 'log';
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/E.png", "-dpng", "-r500")

%% Evaluate control robustness in presence of noise in dynamics

% Number of trials to compute
num_trials = 100;

% Allocate cell array to contain trajectories
x_list = cell(num_trials, 1);

% Noise standard deviation
sigma = 1e1;

% Simulate all the trajectories with noise
for n = 1:num_trials
    % Allocate array for trajectory
    x = cell(length(sol.x), 1);
    
    % Initialize trajectory initial state
    x{1} = x_0;

    % Compute trajectory with computed control sequence with noise
    for k = 1:length(sol.x)-1
        noise = normrnd(0.0, sigma, length(u_max));
        x{k+1} = x{k} + dyn.F(x{k}, sol.u{k} + noise) .* sol.dt;
    end
    
    % Add trajectory to list of computed trajectories
    x_list{n} = x;
end

% Plot the angle trajectories
figure;
pbaspect([5 3 1])
grid on;
hold on;
for n = 1:num_trials
    % Extract the pendulum angle information from the solutions
    theta = zeros(1, length(sol.x));
    theta_dot = zeros(1, length(sol.x));
    for k = 1:length(sol.x)
        theta(k) = x_list{n}{k}(1);
        theta_dot(k) = x_list{n}{k}(2);
    end
    
    % Plot the trajectory
    plot(sol.t, theta)
end
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Pendulum Angle [deg]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/theta-noise-1e1.png", "-dpng", "-r500")

%% Utilize Feedback to adjust control in presence of disturbances

% Number of trials to compute
num_trials = 100;

% Allocate cell array to contain trajectories
x_list = cell(num_trials, 1);

% Noise standard deviation
sigma = 1e1;

% Simulate all the trajectories with noise
for n = 1:num_trials
    % Allocate array for trajectory
    x = cell(length(sol.x), 1);
    
    % Initialize trajectory initial state
    x{1} = x_0;

    % Compute trajectory with updating control sequence with noise
    for k = 1:length(sol.x)-1
        noise = normrnd(0.0, sigma, length(u_max));
        
        % Compute control correction for disturbance using feedback term
        du = -inv(sol.Q_uu{k}) * sol.Q_ux{k} * (x{k} - sol.x{k});
        
        x{k+1} = x{k} + dyn.F(x{k}, sol.u{k} + du + noise) .* sol.dt;
    end
    
    % Add trajectory to list of computed trajectories
    x_list{n} = x;
end

% Plot the angle trajectories
figure;
pbaspect([5 3 1])
grid on;
hold on;
for n = 1:num_trials
    % Extract the pendulum angle information from the solutions
    theta = zeros(1, length(sol.x));
    theta_dot = zeros(1, length(sol.x));
    for k = 1:length(sol.x)
        theta(k) = x_list{n}{k}(1);
        theta_dot(k) = x_list{n}{k}(2);
    end
    
    % Plot the trajectory
    plot(sol.t, theta)
end
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Pendulum Angle [deg]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
%print("~/Dropbox/gatech_classes/ae4803/hw/hw1/report/fig/inverted-pendulum/theta-robust-1e1.png", "-dpng", "-r500")

return;
