function QP = QP_BuildCosts(QP, CM_ades, FP_ades, TR_ades)

%
% QP_BuildCosts.m  -  Build cost function for instantaneous QP of
%   the humanoid model
%
% Inputs:
% QP: QP object (custom)
% MdlParams: robot model parameter structure
% CM_ades: desired CoM acceleration
% FP_des:  desired swing foot point acceleration
% TR_ades: desired hip angular acceleration
% 
% Output:
% QP: QP object with cost terms H and f created or updated


%% Compute H and f terms of swing foot tracking 
% ---------------------------------------------

% Theory:
% (1) Given desired accelerations FP_ades and a weighing matrix Wfp, 
%     the robot joint accelerations are sought that minimize the weighed cost 
%     norm(FP_ades-FP_a)^2_W, where FP_a is the resulting foot point
%     acceleration.
%
%     This minimization is defined as min over FP_a of ||FP_ades-FP_a||^2_W 
%      = min over FP_a of FP_ades^T*Wfp*FP_ades - 2*FP_ades^T*Wfp*FP_a + FP_a^T*Wfp*FP_a
%
%     Since FP_ades is an input over which no optimization happens, the
%     first term is a constant bias in the optimization without changing
%     the location of the minimum. Hence, finding the minimimum is equal to
%     min over FP_a of FP_a^T*Wfp*FP_a - 2*FP_ades^T*Wfp*FP_a
%
% (2) To resolve FP_a, the fundamental relationship FP_v=J*dq between task 
%     space (FP_v) and joint space velocities (dq) is used, where J is the
%     Jacobian relating the two. Hence, 
%     FP_a = d/dt (J*dq) = J*ddq + dJ*dq,
%     
% (3) Substituting this expression for FP_a into the two terms of the
%     minimization problem yields:
%     
%     (a) FP_a^T*Wfp*FP_a = (J*ddq+dJ*dq)^T*Wfp*(J*ddq+dJ*dq) = ...
%                     = ddq^T*J^T*Wfp*J*ddq + 2*dq^T*dJ^T*Wfp*J*ddq + dq^T*dJ^T*Wfp*dJ*dq
%         where W^T=W was used (symmetric cost matrix)
%
%     (b) -2*FP_ades^T*Wfp*FP_a = -2*FP_ades^T*Wfp*(J*ddq+dJ*dq) 
%                            = -2*FP_ades^T*Wfp*J*ddq -2*FP_ades^T*Wfp*dJ*dq
%
% (4) Substituting these terms, the minimization problem transforms into 
%     min_ddq of ddq^T*J^T*Wfp*J*ddq + 2*(dq^T*dJ^T-FP_ades^T)*Wfp*J*ddq +
%     (dq^T*dJ^T-2*FP_ades^T*Wfp*dJ*dq
%     where min over FP_a equals min over ddq was used, as the second term
%     in FP_a=J*ddq+dJ*dq is constant for the instantaneous optimization
%
% (5) The last term is a constant in the search over ddq, and the minimzation 
%     problem simplifies to 
%     min_ddq of ddq^T*J^T*Wfp*J*ddq + 2*(dq^T*dJ^T-FP_ades^T)*Wfp*J*ddq
%
% (6) This minimization problem is equla to the standard QP, 
%     min_x of 1/2*x^T*H*x + f^T*x, where 
%     x = ddq, 
%     H = J^T*Wfp*J, and
%     f = J^T*Wfp*(dJ*dq-FP_ades)
%

Jfp     = QP.Kin.Jfp;             % assign foot point Jacobian
dJfpxdq = QP.Kin.dJfpxdq;         % assign related term dJ*dq

Wfp = diag([1000 1000]);          % swing weight matrix with dimension 2x2 (xFP and yFP)      
Hfp = Jfp'*Wfp*Jfp;               % block of H related to swing
ffp = Jfp'*Wfp*(dJfpxdq-FP_ades); % vector component of f related to swing


%% Compute H and f terms of torso tracking 
% ----------------------------------------

% Theory:
% (1) Given desired acceleration TR_ades of the hip joint and a weight wtr, 
%     the robot joint accelerations ddq is sought that minimizes the weighed cost 
%     norm(TR_ades-TR_a)^2_wtr, where TR_a is the hip joint acceleration.
%
%     This minimization is defined as min over TR_a of ||TR_ades-TR_a||^2_Wtr 
%      = min over TR_a of TR_ades^T*wtr*TR_ades - 2*TR_ades^T*wtr*TR_a + TR_a^T*wtr*TR_a
%
%     Since TR_ades is an input over which no optimization happens, the
%     first term is a constant bias in the optimization without changing
%     the location of the minimum. Hence, finding the minimimum is equal to
%     min over TR_a of TR_a^T*wtr*TR_a - 2*TR_ades^T*wtr*TR_a
%
% (2) The acceleration ddq3 is related to the joint acceleration vector ddq 
%     by TR_a = S*ddq, where S is simply the selection matrix S=[0 0 1 0 0].
%     
% (3) Substituting this expression for TR_a into the two terms of the
%     minimization problem yields:
%     
%     (a) TR_a^T*wtr*TR_a = ddq^T*S^T*wtr*S^T*ddq
%         
%     (b) -2*TR_ades^T*wtr*TR_a = -2*TR_ades^T*wtr*S*ddq 
%                            
% (4) Thus, the minimization problem becomes 
%     min over ddq of ddq^T*S^T*wtr*S*ddq - 2*TR_ades^T*w*S*ddq 
%     
% (5) This minimization problem is equal to the standard QP, 
%     min_x of 1/2*x^T*H*x + f^T*x, where 
%     x = ddq, 
%     H =  S^T*wtr*S, and
%     f = -S^T*wtr*TR_ades

S   = [0 0 1 0 0];     % selection 'matrix'
wtr = 1;               % weight 'matrix'
Htr =  S'*wtr*S;       % H related to trunk balance
ftr = -S'*wtr*TR_ades; % vector component of f related to torso


%% Compute H and f terms of joint torques costs
% ---------------------------------------------

Htau = zeros(5,5); % H related to torque generation
ftau = zeros(5,1); % f related to torque generation


%% Compute H and f terms of CoM tracking
% --------------------------------------

% Theory:
% (1) Given desired accelerations CM_ades and a weighing matrix Wcom, 
%     the robot ground reaction forces are sought that minimizes the weighed 
%     cost "norm(CM_ades-CM_a)^2_Wcom", where CM_a is the actual CoM acceleration.
%
%     This minimization is defined as min over CM_a of ||CM_ades-CM_a||^2_W 
%      = min over CM_a of CM_ades^T*Wcom*CM_ades - 2*CM_ades^T*Wcom*CM_a + CM_a^T*W*CM_a
%
%     Since CM_ades is an input over which no optimization happens, the
%     first term is a constant bias in the optimization without changing
%     the location of the minimum. Hence, finding the minimimum is equal to
%     min over CM_ades of CM_ad^T*W*CM_a - 2*CM_ades^T*Wcom*CM_a
%
% (2) The acceleration CM_a is related to the ground reaction force F by
%     CM_a = 1/m * F + gVec, where g is the gravitational acceleration vector [0 -g]' 
%     
% (3) Substituting this expression for CM_a into the two terms of the
%     minimization problem yields:
%     
%     (a) CM_a^T*Wcom*CM_a = F^T*Wcom/m^2*F + 2*gVec^T*Wcom/m*F + gVec^T*Wcom*gVec
%         where Wcom^T=Wcom was used (symmetric cost matrix)
%
%     (b) -2*CM_ades^T*Wcom*CM_a = -2*CM_ades^T*Wcom/m*F -2*CM_ades^T*Wcom*gVec
%
% (4) Substituting these terms, the minimization problem transforms into 
%     min over F of F^T*Wcom/m^2*F + 2*gVec^T*Wcom/m*F + gVec^T*Wcom*gVec -2*CM_ades^T*Wcom/m*F -2*CM_ades^T*Wcom*gVec
%
% (5) The third and the last term are constant in the search over F, and the minimzation 
%     problem simplifies to 
%     min over F of F^T*Wcom/m^2*F + 2*(gVec^T-CM_ades^T)*Wcom/m*F
%
% (6) This minimization problem is equal to the standard QP, 
%     min_x of 1/2*x^T*H*x + f^T*x, where 
%     x = F, 
%     H = W/m^2, and
%     f = W/m*(gVec-CM_ades)

Wcom = diag([1 100]);                         % weights on tracking horizontal and vertical CoM acceleration
Hcom = Wcom/(QP.Dyn.m)^2;                     % block of H related to GRF 
fcom = Wcom/QP.Dyn.m*(QP.Dyn.gVec - CM_ades); % portion of f related to GRF


%% Compose Cost Function
% ----------------------

% The quadratic term cost matrix has blockdiagonal form  
%
% H = [ Hfp+Htr |  0   |   0 ;
%          0    | Htau |   0 ;
%          0    |  0   | Hcom];
%
% related to the vector [ddq, tau, F]' over which the QP is minimizing.
% Note, costs related to the foot point and trunk are costs over the
% joint accelerations ddq, and related H and f terms are added. 

QP.H = blkdiag( Hfp+Htr, Htau, Hcom); % quadratic terms on [ddq, tau, GRF] 
QP.H = (QP.H+QP.H')/2;                % correct assymetry of H due to round off errors.  
QP.f = [ffp+ftr; ftau; fcom];         % linear terms on [ddq, tau, GRF]

