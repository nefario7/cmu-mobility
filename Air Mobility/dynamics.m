function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************

%     Return states
%     current_state.pos = state(1:3);
%     current_state.vel = state(4:6);
%     current_state.rot = state(7:9);
%     current_state.omega = state(10:12);
    % Linear Acceleration
    R = eul2rotm(state(7:9)',"XYZ");
    ddx_ac = (R * [0; 0; F]) / params.mass - [0; 0; params.gravity];
    
    % Angular Acceleration
    theta = state(7,1);
    phi = state(8,1);
    theta_dot = state(10, 1);
    phi_dot = state(11, 1);
    psi_dot = state(12, 1);
    r = [cos(theta) 0 (-cos(phi)*sin(theta)); 0 1 sin(phi); sin(phi) 0 (cos(phi)*cos(theta))];
    w = r * [theta_dot; phi_dot; psi_dot];

    ddw_ac = params.inertia \ (M - cross(w, params.inertia * w));

    % Linear Velocity
    dx_ac = state(4:6);

    % Angular Velocity
    dw_ac = state(10:12);

    state_dot(1:3) = dx_ac;         % Velocity
    state_dot(4:6) = ddx_ac;        % Acceleration
    state_dot(7:9) = dw_ac;         % Omega
    state_dot(10:12) = ddw_ac;      % Alpha
    state_dot(13:16) = rpm_motor_dot;

    state_dot = state_dot';
end