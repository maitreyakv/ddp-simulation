% Copyright (C) 2019 Maitreya Venkataswamy - All Rights Reserved

% Class for dynamics of inverted pendulum system that inherits from 
% abstract Dynamics class.
classdef InvertedPendulumDynamics < Dynamics
    properties
        m; % mass pendulum
        l; % length of pendulum
        b; % damping coefficient
        I; % moment of intertia of pendulum
    end
    
    methods
        % Constructor
        function obj = InvertedPendulumDynamics(m, l, b)
            obj.m = m;
            obj.l = l;
            obj.b = b;
            obj.I = m .* l.^2;
        end
        
        % Equations of motion in state space representation
        function dxdt = F(obj, x, u)
            theta_ddot = (u - obj.b .* x(2) - obj.m .* 9.81 ...
                                           .* obj.l .* sin(x(1))) ./ obj.I;
                                       
            dxdt = [x(2); theta_ddot];
        end
        
        % Linearized equations of motion (determined using symbolic math)
        function Phi = Phi(obj, x, ~, dt)
            F_x = zeros(numel(x));
            
            F_x(1,1) = 0.0;
            F_x(1,2) = 1.0;
            
            F_x(2,1) = -(9.81 .* obj.l .* obj.m .* cos(x(1))) ./ obj.I;
            F_x(2,2) = -obj.b ./ obj.I;
            
            Phi = eye(numel(x)) + F_x .* dt;
        end
        function beta = beta(obj, x, ~, dt)
            F_u = zeros(numel(x), 1);
            
            F_u(1) = 0.0;
            F_u(2) = 1 ./ obj.I;
                 
            beta = F_u .* dt;
        end
    end
end

