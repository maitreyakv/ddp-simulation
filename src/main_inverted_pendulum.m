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
fps = 30;

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
    
    % Apply grid to background of frame
    grid on;

    % Render the frame
    drawnow;
    
    % Record the frame and store in the structure
    frames(n) = getframe(fig);
    
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
hold on;
plot(sol.t, rad2deg(theta), "k", "LineWidth", 2);
grid on;
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Pendulum Angle [deg]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";

% Plot trajectory in state space
figure;
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

% Plot control sequence
figure;
hold on;
plot(sol.t, u, "b", "LineWidth", 2);
grid on;
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Control Input [N]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";

% Plot cost function vs iteration
figure;
hold on;
plot(1:length(sol.J), sol.J, "r", "LineWidth", 2);
grid on;
xlabel("DDP Iteration [-]", "Interpreter", "latex", "FontSize", 20);
ylabel("Cost Function [-]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
ax.YScale = 'log';

return;
