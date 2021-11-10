function  [MPC, CM_pdes, CM_vdes, CM_ades] = MPC_TrackLIPM(CM_p, CM_v, mParams, MPC, tR, nt, tIdx) 

% time horizon
MPC.N = floor(tR/MPC.dt); %[# of samples] horizon length
if MPC.N==0, MPC.dt=tR; MPC.N=1; end

% cost and constraint matrices given current time horizon
MPC=MPC_CostMatrices(MPC);   MPC=MPC_ConstraintMatrices(MPC); 

% reference signal of prediction horizon
yRef_pred = MPC.yRef(MPC.ny*tIdx + (1:MPC.ny*MPC.N));

% update current CoM state of humanoid 
MPC.xVec(tIdx,:) = [CM_p(1) CM_v(1)]; 
MPC.yVec(tIdx,:) = (MPC.Cr*(MPC.xVec(tIdx,:))')';

% compute function F = F1*x(k) + F2*yRef_pred
MPC.F = MPC.F1*MPC.xVec(tIdx,:)' + MPC.F2*yRef_pred;

% compute B term of inequality constrained Aineq*u=Bineq
MPC.Bineq = MPC.G1*MPC.xVec(tIdx,:)' + MPC.G2*MPC.uPrev + MPC.G3;

% compute optimal control
[uOpt,~,exitflag] = quadprog(MPC.H,MPC.F, MPC.Aineq,MPC.Bineq, [],[], MPC.uMinVec,MPC.uMaxVec, [],MPC.options); % run quadratic program
if exitflag==-2 % assign PD inputsif QP does not converge 
  uFB = max(mParams.pMin,min(mParams.pMax, 10*(MPC.yRef(end-1)-MPC.xVec(tIdx,1)) + 1*(MPC.yRef(end)-MPC.xVec(tIdx,2)) ));
  uOpt=uFB*ones(MPC.N,1); 
end 

% current index
fprintf('Complete: %2.0f%%, MPC-QP exit flag: %d, ', tIdx/(nt-1)*100, exitflag)

% extract first control input
u = MPC_SelectionMatrix(1,MPC.nu,MPC.N) * uOpt;

% update control vector
MPC.uVec(tIdx,:) = u';

% store as last control input
MPC.uPrev = u;

% resulting desired CoM Motion in x direction
CM_pdes = MPC.xVec(:,1)';
CM_vdes = MPC.xVec(:,2)';
CM_ades = mParams.g/mParams.y0 * (CM_pdes-MPC.uVec');

% augment with desired CoM motion in y direction ("no motion")
CM_pdes(2,:) = mParams.y0; % desired height
CM_vdes(2,:) = 0;          % desired vertical velocity
kp_com = 1000; kd_com=100; % PD-gains for maintaining height
CM_ades(2,:) = 0 + kp_com*(CM_pdes(2,tIdx)-CM_p(2)) + kd_com*(CM_vdes(2,tIdx)-CM_v(2)); % zero desired vertical acceleration augmented w/ PD tracking

  