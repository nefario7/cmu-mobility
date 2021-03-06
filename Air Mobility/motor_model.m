function [F_motor, M_motor, rpm_motor_dot] = motor_model(F, M, motor_rpm, params)

% Input parameters
% 
%   F,M: required force and moment
%
%   motor_rpm: current motor RPM [4x1 vector for angular velocities of all ]
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   F_motor: Actual thrust generated by Quadcopter's Motors
%
%   M_motor: Actual Moment generated by the Quadcopter's Motors
%
%   rpm_dot: Derivative of the RPM
%
%************ MOTOR MODEL ************************

% Write code here
ct = params.thrust_coefficient;
cq = params.moment_scale;
d = params.arm_length;

% Force and Moment Transformation matrix 
K = [ct ct ct ct;
    0 d*ct 0 -d*ct;
    -d*ct 0 d*ct 0;
    -cq cq -cq cq];

a = 30978934.324659231722429;
b = 561721383.94667691246471;
c = 1822290254.391719513084044;
K_inv = [
    a 0 -b -c;
    a b 0 c;
    a 0 b -c;
    a -b 0 c];

w_des_square = K_inv * [F; M(1:3,1)];
w_des = sqrt(w_des_square);
w_in = motor_rpm;

w_act = w_des - w_in;

% Limit the motor rpm within the maximum and minimum values
w_motor = real(w_in);
w_motor (w_motor > params.rpm_max) = params.rpm_max;
w_motor (w_motor < params.rpm_min) = params.rpm_min;

fm_motor = K * (w_motor .^ 2);
F_motor = fm_motor(1, 1);
M_motor = fm_motor(2:4, 1);
% disp("Forces and Moments: " + F_motor + " | " + M_motor)

% RPM Dot
rpm_motor_dot = w_act * params.motor_constant;

end
