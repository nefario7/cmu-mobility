function HMD = SIM_SimulateHumanoid(HMD, tx, tauCmd, dt, ntVec)

% SIM_SimulateHumanoid.m  -  Simulate humanoid in lieu of actual robot.
%
% In:
% HMD    : humanoid object (custom)
% tx     : time index
% tauCmd : commanded joint torques at time index tx
% dt     : simulation step width 
%
% Out:
% HMD    : humanoid object (modified)
% 
% H. Geyer, Nov 2018
%

%% Forward Dynamics
% -----------------
%
% Theory:
% The equations of motion are M(q)*ddq + h(q,dq) = tau with h(q,dq) =
% C(q,dq) + N(q). M is positive and symmetric matrix, and the resulting
% accelerations resolve to
%  
% ddq(tx) = M(q(tx))^(-1)*( tau(tx)-h(q(tx),dq(tx)) ).
% 
% With the humanoid joint accelerations resolved, the joint velocities and 
% positions follow from plain integration. Simple Euler integration yields
%
% dq(tx+1) = dq(tx) + ddq(tx)*dt
%  q(tx+1) =  q(tx) +  dq(tx)*dt
%

% current time kinematics and dynamics terms
[~, ~, Jcm, dJcmxdq, Jfp, dJfpxdq] = SIM_ComputeHumanoidKinematics(HMD.q(:,tx), HMD.dq(:,tx));  % CoM and FP Jacobian and related terms dJ*dq    
[M,h] = SIM_ComputeHumanoidDynamics(HMD.q(:,tx), HMD.dq(:,tx));                                 % mass matrix M and vector h = C*dq+N 


%% Current time acclerations
% --------------------------

% !! Disturbance Force to CoM and Random Torque Noise implemented here !!
if tx==20 
    DistImpulse = 0; %[Ns] disturbance impulse
    FxDist = DistImpulse/dt; % [N] horizontal disturbance force corresponding to disturbance impulse 
    tauDist = Jcm'*[FxDist; 0]; % equivalent disturbance joint torques 
else
    tauDist = 0* (rand(5,1)-0.5)*10; % assign torque incvluding random noise of up to +-5Nm
end    
HMD.tau(:,tx)  = tauCmd;
HMD.ddq(:,tx)  = M\( (HMD.tau(:,tx)+tauDist) -h);         % joint accelerations from forward dynamics
HMD.CM_a(:,tx) = Jcm*HMD.ddq(:,tx) + dJcmxdq;             % CoM acceleration from ddr = J*ddq + dJ*dq
HMD.FP_a(:,tx) = Jfp*HMD.ddq(:,tx) + dJfpxdq;             % FP acceleration from ddx = J*ddq + dJ*dq
HMD.GRF(:,tx)  = HMD.Dyn.m*(HMD.CM_a(:,tx)-HMD.Dyn.gVec); % Ground reaction forces


%% Positions and velocities at time tx+1
% --------------------------------------
if tx<ntVec 
    
    % Euler integration
    HMD.dq(:,tx+1) = HMD.dq(:,tx) + HMD.ddq(:,tx)*dt;        
    HMD.q(:,tx+1)  = HMD.q(:,tx)  + HMD.dq(:,tx)*dt;

    % resulting CoM positon and velocity in time step tx+1 
    [CM_p, CM_v] = SIM_ComputeHumanoidKinematics(HMD.q(:,tx+1), HMD.dq(:,tx+1));
    HMD.CM_p(:,tx+1) = CM_p;
    HMD.CM_v(:,tx+1) = CM_v; 

end






