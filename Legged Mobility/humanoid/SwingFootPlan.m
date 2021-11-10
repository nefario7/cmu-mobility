function [FP_pos, FP_vel, FP_acc] = SwingFootPlan(HMDparams, q0, xf, tS, dt)

% SwingFoot_Plan  -  Plan foot point (FP) trajectory of swing leg.
%                    The trajectory is realized as quadratic splines in
%                    x and y with 3 conditions each: (x0,dx0,xf) and
%                    (y0,dy0,yf=0).
%
%
%             |                    |
%             |                    |
%             |                    |
%             o        ->          o
%            / \                  / \
%           /   \                /   \
%     o----o     o              o     o
%  (x0,y0)       |             /       \
%  (dx0,dy0)     |            /         \
%     -----------o-----------o-----------o------
%                |->x                 (xf,yf=0)
% 
%   (Note: xf is the step length, Lstep)
%
% Inputs:
% q0: joint angles at swing initiation (NOTE: Joint angles given with respect to WORLD FRAME!)
% xf: step length from origin of stance foot
% tS: step time
% dt: sample time
%
% Outputs:
%


%% Initial Conditions (x0,dx0, y0,dy0) and Swing Time Vector
% ----------------------------------------------------------

% initial swing foot location from forward kinematics from stance 
% foot point to swing foot point 
x0 = HMDparams.l1*cos(q0(1)) +HMDparams.l2*cos(q0(2)) +HMDparams.l4*cos(q0(4)) +HMDparams.l5*cos(q0(5));
y0 = HMDparams.l1*sin(q0(1)) +HMDparams.l2*sin(q0(2)) +HMDparams.l4*sin(q0(4)) +HMDparams.l5*sin(q0(5));

% assumed initial swing foot velocity
dx0 = 0;    % [m] no slipping
dy0 = 0.75; % [m/s] (arbirary, set to higher values for more leg clearance) 

% time vector (column vector)
tVec = (0:dt:tS)';  ntVec = length(tVec);

%% Trajectories: Quadratic splines of form f(t) = c2*t^2 + c1*t + c0
% (Initial conditions require c0 to be initial position and c1 to be initial velocity)
% ------------------------------------------------------------------------------------
c2=(xf-x0-dx0*tS)/tS^2;                                                              % set constants of x-spline according to constraints
FP_posx=c2*tVec.^2+dx0*tVec+x0;  FP_velx=2*c2*tVec+dx0;  FP_accx=2*c2*ones(ntVec,1); % compute x-trajectory
c2=(0-y0-dy0*tS)/tS^2;                                                               % set constants of y-spline according to constraints
FP_posy=c2*tVec.^2+dy0*tVec+y0;  FP_vely=2*c2*tVec+dy0;  FP_accy=2*c2*ones(ntVec,1); % compute y-trajectory
FP_pos=[FP_posx FP_posy]';  FP_vel=[FP_velx FP_vely]';  FP_acc=[FP_accx FP_accy]';   % assign position, velocity and acceleration trajectories




