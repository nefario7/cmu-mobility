function  FP_acc = PD_TrackSwingFP(q, dq, MdlParams, FP_acc, tIdx, FPpos, FPvel)

% PD_TrackSwingFP.m  -  Track the nominal swing foot plan using 
%                       computed torque control.
%


% precompute sine and cosine functions
c = cos(q); s = sin(q);

% current swing foot point position (measured from stance leg foot point)
xFP  = [  MdlParams.l1*c(1)+MdlParams.l2*c(2)+MdlParams.l4*c(4)+MdlParams.l5*c(5); ...
          MdlParams.l1*s(1)+MdlParams.l2*s(2)+MdlParams.l4*s(4)+MdlParams.l5*s(5)];
  
% current swing foot point velocity    
dxFP = [-MdlParams.l1*s(1)*dq(1)-MdlParams.l2*s(2)*dq(2)-MdlParams.l4*s(4)*dq(4)-MdlParams.l5*s(5)*dq(5); ...
         MdlParams.l1*c(1)*dq(1)+MdlParams.l2*c(2)*dq(2)+MdlParams.l4*c(4)*dq(4)+MdlParams.l5*c(5)*dq(5)];

% define proportional and derivative gain
kp_sw = 100;  kd_sw = 10;

% compute desired acceleration (nominal acceleration as feedforward term plus PD-feedback tracking of desired motion
FP_acc(:,tIdx) = FP_acc(:,tIdx) + kp_sw*(FPpos-xFP) + kd_sw*(FPvel-dxFP);




     