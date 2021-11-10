function QP = QP_UpdateDynamicsModel(QP, q,dq)

% 
% QP_UpdateDynamicsModel  -  Update kinematics and dynamics model of humanoid for QP
%                            optimization. 
%
% NOTE: The update needs to be performed before cost and constraint terms
% are being computed.
%
% Updates include the mass matrix M, the Coriolis and gravitational term h = C*dq+N,
% the Jacobians Jcm and Jfp mapping to the robot center of mass (CM) and to the 
% swing foot point (FP), respectively, and the related terms dJ*dq used in the constraint
% and cost functions.
%
% In: 
% QP: QP object
%
% Out:
% QP: QP object (updated)
%
% H. Geyer, Nov 2018
%

% compute kinematics and dynamics terms
% IMPORTANT: The function ComputeKinAndDynTerms was generated in the symbolics subfolder from
% symbolic math scripts that have been converted into this matlab function using the specific
% model parameters in \symbolics\DefineHumanoidParams.m. If these model
% parameters are changed, the function QP_ComputeKinAndDynTerms has to be
% regenerated!
[M, h, Jcm, Jfp, dJcmxdq, dJfpxdq] = QP_ComputeKinAndDynTerms(q, dq);
    
% assign terms to QP object
QP.Dyn.M = M;
QP.Dyn.h = h;
QP.Kin.Jcm = Jcm;
QP.Kin.Jfp = Jfp;
QP.Kin.dJcmxdq = dJcmxdq;
QP.Kin.dJfpxdq = dJfpxdq;

