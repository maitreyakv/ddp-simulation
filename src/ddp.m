% Copyright (C) 2019 Maitreya Venkataswamy - All Rights Reserved

%% Differential Dynamic Programming Algorithm Function

% Performs Differential Dynamic Programming Algorithm to produce a
% locally optimal control sequence.
%
% Inputs
% 
% x_0      : initial state vector
% x_star   : target state vector
% t_f      : time horizon
% N        : number of discretizations of time
% dyn      : instance of Dynamics class
% cost     : instance of Cost class
% u_max    : maximum magnitude of control, used for clamping
% num_iter : number of DDP iterations
% alpha    : DDP learning parameter for line searching
%
% Outputs
% 
% sol : structure with solution components
%
function sol = ddp(x_0, x_star, t_f, N, dyn, cost, u_max, num_iter, alpha)
    %% Allocate arrays for DDP

    % Time stamp array and timestep
    t = linspace(0.0, t_f, N);
    dt = t(2) - t(1);
    
    % Cost history
    J = zeros(num_iter, 1);
    
    % Control energy history
    E = zeros(num_iter, 1);
    
    % State trajectory
    x = cell(N, 1);
    x_new = cell(N, 1);
    x{1} = x_0;
    x_new{1} = x_0;
    
    % Control Input trajectory
    u = cell(N, 1);
    
    % Value function and derivatives
    V = zeros(N, 1);
    V_x = cell(N, 1);
    V_xx = cell(N, 1);
    
    % State action value derivatives
    Q_x = cell(N, 1);
    Q_u = cell(N, 1);
    Q_xx = cell(N, 1);
    Q_uu = cell(N, 1);
    Q_xu = cell(N, 1);
    Q_ux = cell(N, 1);

    %% Initialize DDP with a random input sequence

    fprintf("initializing random control sequence...\n")
    
    % Generate random control sequence
    for k = 1:N-1
        u{k} = 2 .* u_max .* rand(length(u_max), 1) - u_max;
    end
    u{N} = zeros(numel(u_max), 1);
    
    fprintf("generating initial trajectory...\n")
    
    % Generate initial trajectory using random control sequence
    for k = 1:N-1
        x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}) .* dt;
    end
    
    %% Perform main DDP iterations on the trajectory and input sequence
    
    fprintf("beginning DDP...\n")
    
    for i = 1:num_iter
        fprintf("DDP iteration %d out of %d...\n", i, num_iter);
        
        % Update control sequence from previous iteration
        if i > 1
            for k = 1:N-1
                % Compute control update feed-forward and feed-back
                du_ff = -inv(Q_uu{k}) * Q_u{k};
                du_fb = -inv(Q_uu{k}) * Q_ux{k} * (x_new{k} - x{k});
                
                % Limit feed forward control modification with clamping
                for m = 1:numel(u_max)
                    du_ff(m) = min(u_max(m), max(-u_max(m), ...
                                         du_ff(m) + u{k}(m))) - u{k}(m);
                end
                
                % Update control
                u{k} = u{k} + alpha .* (du_ff + du_fb);
                
                % Compute next state in trajectory with new control
                x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}) .* dt;
                
                % Return error if problem with trajectory
                if isnan(x_new{k+1})
                    sol = assemble_solution(x, u, t, J, E, 1);
                    return
                end
            end
        end
        
        % Update the current trajectory
        x = x_new;
        
        % Compute total cost
        J(i) = cost.phi(x{N}, x_star);
        for k = 1:N-1
            J(i) = J(i) + cost.L(x{k}, u{k}, dt);
        end
        
        % Compute control energy usage
        for k = 1:N-1
            E(i) = E(i) + 0.5 .* u{k}.' * u{k} .* dt;
        end
        
        % Compute terminal value function and derivatives
        V(N) = cost.phi(x{N}, x_star);
        V_x{N} = cost.phi_x(x{N}, x_star);
        V_xx{N} = cost.phi_xx(x{N}, x_star);
        
        % Perform backwards pass
        for k = N-1:-1:1
            % Compute state-action value function derivatives
            Q_x{k} = cost.L_x(x{k}, u{k}, dt) + ...
                     dyn.Phi(x{k}, u{k}, dt).' * V_x{k+1};
            Q_u{k} = cost.L_u(x{k}, u{k}, dt) + ...
                     dyn.beta(x{k}, u{k}, dt).' * V_x{k+1};
            Q_xx{k} = cost.L_xx(x{k}, u{k}, dt) ...
                      + dyn.Phi(x{k}, u{k}, dt).' * V_xx{k+1} ...
                      * dyn.Phi(x{k}, u{k}, dt);
            Q_uu{k} = cost.L_uu(x{k}, u{k}, dt) ...
                      + dyn.beta(x{k}, u{k}, dt).' * V_xx{k+1} ...
                      * dyn.beta(x{k}, u{k}, dt);
            Q_xu{k} = cost.L_xu(x{k}, u{k}, dt) ...
                      + dyn.Phi(x{k}, u{k}, dt).' * V_xx{k+1} ...
                      * dyn.beta(x{k}, u{k}, dt);
            Q_ux{k} = cost.L_ux(x{k}, u{k}, dt) ...
                      + dyn.beta(x{k}, u{k}, dt).' * V_xx{k+1} ...
                      * dyn.Phi(x{k}, u{k}, dt);
               
            % Compute the value function derivatives
            V_x{k} = Q_x{k} - Q_xu{k} * (Q_uu{k} \ Q_u{k});
            V_xx{k} = Q_xx{k} - Q_xu{k} * (Q_uu{k} \ Q_ux{k});
        end
    end
    
    %% Assemble and return solution structure
    
    fprintf("finished DDP, assembling results for post-processing...\n");
    
    % Assemble solution
    sol = assemble_solution(x, u, t, J, E, 0);
end

%% Helper Functions for DDP Algorithm

% Assembles and returns solution structure
%
% Inputs
%
% x     : locally optimal state trajectory
% u     : locally optimal control sequence
% t     : discretized time stamps
% J     : iteration history of cost function
% E     : iteration history of control energy
% error : zero if no error, nonzero else
%
% Outputs
%
% sol : solution structure
function sol = assemble_solution(x, u, t, J, E, error)

    % Solution structure
    sol = struct;
    sol.error = error;
    sol.x = x;
    sol.u = u;
    sol.t = t;
    sol.dt = t(2) - t(1);
    sol.J = J;
    sol.E = E;
    
    return
end