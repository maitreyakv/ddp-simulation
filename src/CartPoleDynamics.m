% Copyright (C) 2019 Maitreya Venkataswamy - All Rights Reserved

% Class for dynamics of cart-pole system that inherits from abstract 
% Dynamics class.
classdef CartPoleDynamics < Dynamics
    properties
        m_c; % mass of pole
        m_p; % mass of cart
        l; % length of pole
    end
    
    methods
        % Constructor
        function obj = CartPoleDynamics(m_c, m_p, l)
            obj.m_c = m_c;
            obj.m_p = m_p;
            obj.l = l;
        end
        
        % Equations of motion in state space representation
        function dxdt = F(obj, x, u)
            s = sin(x(3));
            c = cos(x(3));
            
            lambda = obj.m_c + obj.m_p .* s.^2;
            psi = u + obj.m_p .* s .* (obj.l .* x(4).^2 + ...
                                                9.81 .* c);

            eta = -u .* c - obj.m_p .* obj.l .* x(4).^2 ...
                  .* c .* s - (obj.m_c + obj.m_p) ...
                  .* 9.81 .* s;
            
            z_ddot = psi ./ lambda;
            theta_ddot = eta ./ (obj.l .* lambda);
            
            dxdt = [x(2); z_ddot; x(4); theta_ddot];
        end
        
        % Linearized equations of motion (determined using CAS)
        function Phi = Phi(obj, x, u, dt)
            F_x = zeros(numel(x));
            
            s = sin(x(3));
            c = cos(x(3));
            
            F_x(1,1) = 0.0;
            F_x(1,2) = 1.0;
            F_x(1,3) = 0.0;
            F_x(1,4) = 0.0;
            
            F_x(2,1) = 0.0;
            F_x(2,2) = 0.0;
            F_x(2,3) = (obj.m_p .* c .* (obj.l .* x(4).^2 + ...
                       9.81 .* c) - 9.81 .* obj.m_p .* ...
                       s.^2) ./ (obj.m_c + obj.m_p .* ...
                       s.^2) - (2.0 .* obj.m_p .* c ...
                       .* s .* (u + obj.m_p .* s ...
                       .* (obj.l .* x(4).^2 + 9.81 .* c))) ...
                       ./ (obj.m_c + obj.m_p .* s.^2)^2;
            F_x(2,4) = (2.0 .* obj.l .* obj.m_p .* x(4) .* s) ...
                ./ (obj.m_c + obj.m_p .* s.^2);
            
            F_x(3,1) = 0.0;
            F_x(3,2) = 0.0;
            F_x(3,3) = 0.0;
            F_x(3,4) = 1.0;
            
            F_x(4,1) = 0.0;
            F_x(4,2) = 0.0;
            F_x(4,3) = (u .* s - 9.81 .* c .* ...
                       (obj.m_c + obj.m_p) - obj.l .* obj.m_p ...
                       .* x(4).^2 .* c.^2 + obj.l .* obj.m_p ...
                       .* x(4)^2 .* s.^2) ./ (obj.l ...
                       .* (obj.m_c + obj.m_p .* s.^2)) ...
                       + (2.0 .* obj.m_p .* c .* s ...
                       .* (obj.l .* obj.m_p .* c .* s ...
                       .* x(4).^2 + u .* c + 9.81 ...
                       .* s .* (obj.m_c + obj.m_p))) ./ (obj.l ...
                       .* (obj.m_c + obj.m_p .* s.^2)^2);
            F_x(4,4) = -(2.0 .* obj.m_p .* x(4) .* c ...
                       .* s) ./ (obj.m_c + obj.m_p .* s^2);
            
            Phi = eye(numel(x)) + F_x .* dt;
        end
        function beta = beta(obj, x, ~, dt)
            F_u = zeros(numel(x), 1);
            
            F_u(1) = 0.0;
            F_u(2) = 1.0 ./ (obj.m_c + obj.m_p .* sin(x(3)).^2);
            F_u(3) = 0.0;
            F_u(4) = -cos(x(3)) ./ (obj.l .* (obj.m_c + obj.m_p ...
                     .* sin(x(3)).^2));
                 
            beta = F_u .* dt;
        end
    end
end

