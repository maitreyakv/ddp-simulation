%% Setup

clc;
clear;
close all;
rng(0);

%% Cart-pendulum problem definition

% Initial state
x_0 = [0.0; 0.0; 0.0; 0.0];

% Target state
x_star = [0.0; 0.0; pi; 0.0];

% Time horizon
t_f = 2.0;

% Number of states along trajectory
N = floor(t_f ./ 0.01);

% Maximum magnitude of control
u_max = [10.0];

% Initialize dynamics
fprintf("initializing cart-pole dynamics...\n")
m_c = 1.0;
m_p = 0.01;
l = 0.25;
dyn = CartPoleDynamics(m_c, m_p, l);

% Initialize cost
fprintf("initializing quadratic cost function...\n")
Q_f = [10.0 0 0 0;
       0 10.0 0 0;
       0 0 10.0 0;
       0 0 0 10.0];
R = 0.1;
cost = QuadraticCost(Q_f, R);

% Number of DDP iterations
num_iter = 100;

% DDP learning rate
alpha = 0.1;

% Video framerate
fps = 10;

%% Execution of DDP

fprintf("executing DDP...");

tic;
sol = ddp(x_0, x_star, t_f, N, dyn, cost, u_max, num_iter, alpha);
toc;

%% Begin post-processing of solution

if sol.error == 1
    fprintf("DIVERGENCE ERROR: try decreasing learning rate\n");
    return;
end

% Extract the cart position and pole angle information from the solution
z = zeros(1, length(sol.x));
theta = zeros(1, length(sol.x));
u = zeros(1, length(sol.x));
for k = 1:N
    z(k) = sol.x{k}(1);
    theta(k) = sol.x{k}(3);
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
vid = VideoWriter('cart-pole', 'Uncompressed AVI');
vid.FrameRate = fps;
open(vid);

% Loop over points in time to process frames at
for k = 1:ceil((1.0 ./ fps) ./ sol.dt):N
    % Plot the joint between cart and pole
    plot(z(k), 0.0, 'ro');

    % Apply plot hold
    hold on;

    % Plot the cart
    rectangle('Position', [z(k) - 0.25 .* l, -0.1 .* l 0.5 .* l, 0.2 .* l]);

    % Coordinated of the tip of the pole
    tipz = z(k) + l .* sin(theta(k));
    tipy = -l .* cos(theta(k));

    % Plot the pole
    plot([z(k), tipz], [0.0, tipy], '-b');
    
    % Plot the cart track
    plot(-100:100, -0.1 .* l .* ones(1, length(-100:100)), "-k");
    
    % Remove plot hold
    hold off;

    % Obtain or apply plot limits for consistency across frames
    if k == 1
        xlim([-2 .* l, 2 .* l]);
        axis equal;
        xl = xlim();
        yl = ylim() + 0.5 .* l;
        ylim(yl);
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
    writeVideo(vid, frames(n));
    
    % Display progress to command line
    fprintf("rendering video frame %d out of %d...\n", n, num_frame);
    
    % Increment frame counter
    n = n + 1;
end

% Close the video
close(vid);

% Renable plot visibility to view movie and other plots
set(0,'DefaultFigureVisible','on')

% Create fresh figure object for the movie
figure;

% Render the movie and set to loop for a very large number of times
fprintf("done rendering video frames, presenting video...\n")
movie(gcf, frames, 9999, fps);

%% Plot trajectories

figure;
hold on;
plot(sol.t, z, "k");

figure;
hold on;
plot(sol.t, theta, "k");

figure;
hold on;
plot(z, theta, "k");
plot(x_0(1), x_0(3), '-bo');
plot(x_star(1), x_star(3), '-go')

figure;
hold on;
plot(sol.t, u, "b", "LineWidth", 2);
grid on;
xlabel("Time [s]", "Interpreter", "latex", "FontSize", 20);
ylabel("Control Input [N]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";


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

figure;
hold on;
plot(1:length(sol.E), sol.E, "r", "LineWidth", 2);
grid on;
xlabel("DDP Iteration [-]", "Interpreter", "latex", "FontSize", 20);
ylabel("Control Energy Usage [-]", "Interpreter", "latex", "FontSize", 20);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
ax.YScale = 'log';

return;
