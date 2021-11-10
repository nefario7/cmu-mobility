function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here
x_d = desired_state.acc(1, 1);  %ex_dot_dot + x_dot_dot
y_d = desired_state.acc(2, 1);  %ey_dot_dot + y_dot_dot
psi_d = desired_state.rot(3, 1);    %desired psi
psi_d_dot = desired_state.omega(3, 1);  % desired psi_dot

%% Euler Angles (rot)
R = [sin(psi_d) -cos(psi_d); cos(psi_d) sin(psi_d)];

rot(1:2, 1) = (R * [x_d; y_d]) / params.gravity;
rot(3, 1) = psi_d;

%% Derivatives of Euler Angles (omega)
R_dot = [cos(psi_d) sin(psi_d); -sin(psi_d) cos(psi_d)];    % Rotation Matrix

omega(1:2, 1) = (R_dot * [x_d * psi_d_dot; y_d * psi_d_dot]) / params.gravity; 
omega(3, 1) = psi_d_dot;

end

