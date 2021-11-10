%
% main.m  - Main file for Humanoid controller
%
% H. Geyer, Nov 2018
%


clc

% add mpc, qp and symbolics subfolders to matlab path
addpath([pwd '/mpc'])
addpath([pwd '/qp'])
addpath([pwd '/simulator'])
addpath([pwd '/symbolics'])


% ************ %
% DECLARATIONS %
% ************ %


%% Initial Conditions
% -------------------

% Humanoid joint angles in WORLD FRAME COORDINATES  
q0 = pi/180*[  70; ... % stance leg tibia angle
              100; ... % stance leg femur angle
               90; ... % torso angle
              250; ... % swing leg femur angle
              240];    % swing leg tibia angle

% initial humanoid CoM position
CM_pos0 = SIM_ComputeHumanoidKinematics(q0, [0 0 0 0 0]');

% initial CoM conditions
x0 = CM_pos0(1);
y0 = CM_pos0(2);
dx0 = 0;
          

%% Model Parameters
% -----------------

% general LIPM parameters
LIPMparams.g      =   9.81; %[m/s^2] gravitational acceleration
LIPMparams.y0     =     y0; %[m] constant height of CoM
LIPMparams.lMax   =      1; %[m] maximum leg length
LIPMparams.pMin   =  -0.05; %[m] minimum CoP (measured from ankle joint
LIPMparams.pMax   =   0.20; %[m] maximum CoP
LIPMparams.dt     =   0.01; %[s] sample time

% Humanoid model
HMDparams = DefineHumanoidParams;



    
%% Goal Behavior
% --------------
Lstep = 0.3; %[m] nominal step length
dxE =   0.5; %[m/s] desired walking velocity
    



% ******* %
% PROGRAM %
% ******* %

% Initialize humanoid simulator
SimulatorFigNr = 1;
HMD = SIM_InitHumanoidSimulator(HMDparams,SimulatorFigNr);


%% Create Nominal Plans for Center of Mass (CM), Swing Foot Point (FP), and Trunk (TR)
% -------------------------------------------------------------------------------------

% create nominal stance plan for CoM from LIPM without ankle torque
[CM_pdes, CM_vdes, tS, xT] = LIPM_NominalPlan(x0,dx0, dxE, Lstep, LIPMparams);

% create nominal stance plan for foot point (FP) of swing leg
[FP_pdes, FP_vdes, FP_ades] = SwingFootPlan(HMDparams, q0, CM_pdes(end)+xT, tS, LIPMparams.dt);

% create nominal plan for torso
n=size(FP_pdes,2); TR_pdes=zeros(1,n); TR_vdes=zeros(1,n); TR_ades=zeros(1,n);

% show plan
ShowStepPlan(q0, FP_pdes, CM_pdes, LIPMparams, HMD, SimulatorFigNr)  



%% Track Plans through Stance Phase 
% ---------------------------------

% Initialize MPC tracking of CoM plan with LIPMp
Qx=10; Qdx=1; R=1; % assign costs on position and velocity tracking as well as on CoP control
tR = tS;           % assign time horizon
MPC = MPC_DefineLIPMpMPC(LIPMparams, Qx, Qdx, R, tS, CM_pdes, CM_vdes);  % define MPC problem

% Initialize instantanteous QP tracking of humanoid behavior goals 
QP = QP_InitHumanoidQP(HMDparams);

% define time vector
tVec = 0:LIPMparams.dt:tS; ntVec=length(tVec);

% preallocate memory and initialize humanoid joint trajectories
nq = 5;                                        % number of joints
HMD.q   = NaN(nq, ntVec);  HMD.q(:,1)= q0;            % joint angles
HMD.dq  = NaN(nq, ntVec);  HMD.dq(:,1)= [0 0 0 0 0]';  % joint velocities
HMD.ddq = NaN(nq, ntVec);  HMD.ddq(:,1)= [0 0 0 0 0]';  % joint acceleration

% preallocate memory and initialize humanoid CoM trajectories
HMD.CM_p=NaN(2,ntVec); HMD.CM_p(:,1)=[CM_pdes(1); y0]; % CoM position
HMD.CM_v=NaN(2,ntVec); HMD.CM_v(:,1)=[CM_vdes(1);  0]; % CoM velocity

% preallocate memory of humanoid joint torque and ground reaction force trajectories 
HMD.tau = NaN(nq, ntVec);
HMD.GRF = NaN( 2, ntVec);

% loop through stance phase
for tIdx = 1:ntVec-1 
  
  fprintf('Current stance time: %4.3fs (index: %d), ', tVec(tIdx), tIdx)
  
  % get remaining time in stance
  tR = tS-tVec(tIdx); 
  
  % update desired stance dynamics of CoM using MPC LIPM tracking 
  [MPC, CM_pdes, CM_vdes, CM_ades] = MPC_TrackLIPM(HMD.CM_p(:,tIdx), HMD.CM_v(:,tIdx), LIPMparams, MPC, tR, ntVec, tIdx); 
  
  % update desired acceleration of swing foot using PD tracking of nominal foot point plan
  FP_ades = PD_TrackSwingFP(HMD.q(:,tIdx), HMD.dq(:,tIdx), HMDparams, FP_ades, tIdx, FP_pdes(:,tIdx), FP_vdes(:,tIdx));
  
  % update desired trunk acceleration using PD tracking of desired trunk pitch
  kp_tr=0.5;  kd_tr=0.05;  
  TR_ades(tIdx) = TR_ades(tIdx) +kp_tr*(TR_pdes(tIdx)-HMD.q(3,tIdx))+kd_tr*(TR_vdes(tIdx)-HMD.dq(3,tIdx));
  
  % update humanoid kinematics and dynamics model based on current joint position and velocities 
  QP = QP_UpdateDynamicsModel(QP, HMD.q(:,tIdx), HMD.dq(:,tIdx));
  
  % compute QP cost and constraint terms  
  QP = QP_BuildCosts(QP, CM_ades(:,tIdx), FP_ades(:,tIdx), TR_ades(tIdx));
  QP = QP_BuildConstraints(QP);
  
  % solve QP
  [x, ~,exitflag]=quadprog(QP.H,QP.f, QP.Aineq, QP.bineq, QP.Aeq,QP.beq, QP.lb,QP.ub, [],QP.options);
  fprintf(' Humanoid-QP exit flag: %d.\n', exitflag)
  
  if exitflag==1
      QP.ddq(:,tIdx) = x(1:5);  QP.tau(:,tIdx) = x(6:10);  QP.GRF(:,tIdx) = x(11:12);
  else
      return
  end
  
  % simulate humanoid robot
  HMD = SIM_SimulateHumanoid(HMD, tIdx, QP.tau(:,tIdx), LIPMparams.dt, ntVec);
  
  % animate robot
  SIM_AnimateHumanoid(HMD,HMD.q(:,tIdx), SimulatorFigNr)
  
  %pause
  
end

fprintf('\n\n')



% plot MPC tracking behavior
MPCfigNr = 2;
ShowMPCtrackingBehavior(MPC, LIPMparams, tVec, MPCfigNr)

% plot humanoid QP tracking behavior
ShowHumanoidTrackingBehavior;