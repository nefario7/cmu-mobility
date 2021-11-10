function [x_des, dx_des, tS, xT] = LIPM_NominalPlan(x0,dx0, dxE, L, mParams)

%
% LIPM_Plan.m  - Create a nominal LIPM plan for the current stance leg
% without considering ankle torques (ddx = w^2 * x with w=sqrt(g/y0) )
% The nominal plan assumes going from an arbitrary initial pose of the
% current stance leg to the vertical pose of next stance leg.
%
%              | stance  |
% (x0, dx0) => o----o----o----o => (xf=0, dxf)
%               \   |   / \   |
%                \  |  /   \  |
%                 \ | /     \ |
%                  \|/       \|
%              -----o---------o
%                   | xS | xT |
%                   |    L    |
%                   |-> x coordinate with respect to current stance foot
%
% (Note: x0 can be positive or negative)
%
% The plan computes the transition point xS and the target step point xT
% from considering the balance between kinetic energy and work performed on
% the point mass.
%
% With xS given, the plan computes the resulting trajectories of the
% position x_LIPM and the velocity dx_LIPM until the end of the current stance. 

% Inputs: 
% x0, dx0 : initial position and velocity of CoM
% dxE     : final CoM velocity at vertical pose of next stance leg
% L       : step length
% mParams : LIPM model parameters including graviational acceleration and 
%           constant vertical height
%
% Outputs:
% x_LIPM, dx_LIPM : nominal LIPM trajectories of position and velocity until end of current stance
% tS      : corresponding stance time
% xT      : nominal capture point for reaching velocity dxE at vertical pose of opposite leg



% ************ %
% DECLARATIONS %
% ************ %

% model parameters
w = sqrt(mParams.g/mParams.y0);  % eigenfrequency



% ******* % 
% PROGRAM %
% ******* %

% Calculate transition and target distances (xS and xT)
% given step length L:
% -----------------------------------------------------
% energy balance: W = mg / 2y0 * (xS^2-x0^2-xT^2) !=! m/2 (dxf^2-dx0^2)
% => substitute xT=L-xS and resolve for xS:
xS = L/2 +x0^2/(2*L) + (dxE^2-dx0^2) / (2*L*w^2);

if xS>L, xS=L; end  % limit xS to L if desired speed changes require step lengths beyond L
                    % note: desired speed dxe will not be achieved
xT = L-xS;          % set step target distance xT


% Calculate time at which xS is reached: 
% --------------------------------------
% solve xS = x0 cosh(w*tS) + dx0/w sinh(w*tS) for tS
% (use exponential forms cosh=0.5*(e^wt+e^-wt) and sinh=0.5*(e^wt-e^-wt) 
% and write equation with substitute s = e^wt; then solve resulting 
% quadratic equation in s and substitute back)
% (only one solution is valid)
tS = 1/w * log( (xS+sqrt(xS^2-x0^2+(dx0/w)^2)) / (x0+dx0/w) );


% Generate nominal plan
% ---------------------
tVec    = 0:mParams.dt:tS;                          % time vector from now until stance transition 
x_des   =   x0 * cosh(w*tVec) + dx0/w*sinh(w*tVec); % desired CoM positions
dx_des  = w*x0 * sinh(w*tVec) + dx0  *cosh(w*tVec); % desired CoM velocities









