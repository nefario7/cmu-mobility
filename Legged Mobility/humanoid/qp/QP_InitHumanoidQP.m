function QP = QP_InitHumanoidQP(MdlParams)

% 
% QP_InitHumanoidQP.m  -  Initialize QP for Humanoid tracking of desired
%                         behaviors
%
% In:
% MdlParams: Humanoid model parameters 
% 
% Out:
% QP: QP object (updated)
%
%
% QP is defined as minimization problem 
% min over x of x^T*H*x + f^T*x where x is the input
%
% subject to Aeq*x   = beq   (equality constraints)
%            Aineq*x < bineq (inequality constraints)
%            xmin < x < xmax (bounds)
%

% initialize QP object
QP = [];


%% Set Optimization Options
% -------------------------

QP.options = optimoptions('quadprog','Display','off');


%% Define Permanent Dynamics Terms
% --------------------------------

QP.Dyn.m = MdlParams.m1 + MdlParams.m2 + MdlParams.m3 + MdlParams.m4 + MdlParams.m5;  % total humanoid mass
QP.Dyn.gVec = [0 -MdlParams.g]';                                                      % vector of gravitational acceleration


%% Define Input Bounds
% --------------------

JointAccelLimit = 1000;  % joint acceleration limits [rad/s^2]
TorqueLimits    = [100 300 500 300 100];  % joint torque limits [Nm]
GRFlimits       = 3000;  % ground reaction force limits [N]
QP.lb = [ -JointAccelLimit*[1 1 1 1 1] -TorqueLimits [-GRFlimits         0] ]; % assign lower bounds
QP.ub = [  JointAccelLimit*[1 1 1 1 1]  TorqueLimits [ GRFlimits GRFlimits] ]; % assign upper bounds






