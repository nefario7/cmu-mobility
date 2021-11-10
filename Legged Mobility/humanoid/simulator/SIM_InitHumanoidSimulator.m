function HMD = SIM_InitHumanoidSimulator(HMDparams, FigNr)

% SIM_InitHumanoidSimulator  - Initialize humanoid simulation and animation
%
% In:
% HMDparams : Humanoid parameters
%
% Out:
% HMD : Humanoid object
%
% H. Geyer, Nov 2018
%

% create humanoid object
HMD = [];


%% Assign Humanoid Parameters
% ---------------------------

HMD.params = HMDparams;


%% Define Permanent Dynamics Terms
% --------------------------------

HMD.Dyn.m = HMDparams.m1 + HMDparams.m2 + HMDparams.m3 + HMDparams.m4 + HMDparams.m5;  % total humanoid mass
HMD.Dyn.gVec = [0 -HMDparams.g]';                                                      % vector of gravitational acceleration


%% Define Animation Objects
% -------------------------

figure(FigNr), clf, hold on, axis equal % Initialize figure
HMD.Anim.RLeg  = plot([0 0 0],[0 0 0], 'r', 'LineWidth', 4); % right leg
HMD.Anim.LLeg  = plot([0 0 0],[0 0 0], 'b', 'LineWidth', 4); % left leg
HMD.Anim.Trunk = plot(  [0 0],  [0 0], 'k', 'LineWidth', 8); % trunk
HMD.Anim.CM = plot(0, 0, 'ko', 'MarkerSize',15); % humanoid CoM

