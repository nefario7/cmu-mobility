function [M] = attitude_controller(current_state, desired_state, params, question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
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
%   question: Question number
%
% Output parameters
%
%   M: u2 or moment [M1; M2; M3]
%
%************  ATTITUDE CONTROLLER ************************

if question == 2 || question == 41 || question == 43 || question == 44
    % Example PD gains
    Kpphi = 150;
    Kdphi = 30;
    
    Kptheta = 160;
    Kdtheta = 30;
    
    Kppsi = 80;
    Kdpsi = 17.88;
    
elseif question == 3 || question == 42 || question == 45
    Kpphi = 190;
    Kdphi = 30;
    
    Kptheta = 198;
    Kdtheta = 30;
    
    Kppsi = 80;
    Kdpsi = 17.88;

elseif question == 52 ||  question == 53 || question == 58 || question == 59
%     % Gain Set 1
%     Kpphi = 190;
%     Kdphi = 30;
%     
%     Kptheta = 190;
%     Kdtheta = 30;
%     
%     Kppsi = 70;
%     Kdpsi = 18;

    % Gain Set 2
    Kpphi = 190;
    Kdphi = 30;
    
    Kptheta = 190;
    Kdtheta = 30;
    
    Kppsi = 20;
    Kdpsi = 18;
end

% Gain values for P and D
Kp = [Kpphi; Kptheta; Kppsi];
Kd = [Kdphi; Kdtheta; Kdpsi];

% Errors
error.rot = current_state.rot - desired_state.rot;
error.omega = current_state.omega - desired_state.omega;

% Required Moment
M = params.inertia * (-Kp .* error.rot - Kd .* error.omega);
end

