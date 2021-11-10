function QP = QP_BuildConstraints(QP)

%
% QP_BuildConstraints.m  -  Build constraint terms for instantaneous QP of
%                             the humanoid model
%
% Inputs:
% QP: QP object (custom)
% 
% Output:
% QP: QP object with constraint equation terms Aeq and beq created or updated


% Theory:
% (1) The equations of motion of a kinematic chain are given by
%     M*ddq + C*dq + N = tau, where M is the mass matrix, C is
%     the Coriolis matrix and N is the gravitational vector.
%
% (2) The equations can be realigned as M*ddq -tau = -h with h=C*dq+N,
%     which can be used to define a constraint on the joint accelerations
%     and torques:
%     [M -eye(5)] * [ddq tau]' = -h
%
% (3) A second set of equations of motion is used to constrain the  
%     leg forces F not covered in the first equation set. 
%
%     The equation of motion for the center of mass is given by
%     m*CM_a = F +m*gVec with gVec = [0 -g]'. The CoM acceleration is related to 
%     the joint accelerations by CM_a = d/dt(CM_v) = d/dt(Jcm*q) = Jcm*ddq + dJcm*dq, 
%     where Jcm is the Jacobian mapping the CoM to the joint angles. Combining the two equations 
%     yields: 
%     F + m*gVec = m*(Jcm*ddq + dJcm*dq)
%
% (4) This equation can be realigned to a second constraint on the optimization variable:
%     [m*Jcm  -eye(2)] * [ddq F]' = m*(gVec-dJcm*dq)
%
% (5) Combining the two constraint equations yields
%     [  M    |  -eye(5)   | zeros(5,2)] * [ddq] = [   -h       ]
%     [ m*Jcm | zeros(2,5) |  -eye(2)  ]   [tau]   [m*(gVec-dJcm*dq)]
%                                          [ F ]
%
%     This equation fits the standard equality constraint Aeq * x = beq with
%     x   = [ddq tau F]', 
%     Aeq = [M -eye(5) zeros(5,2); m*Jcm zeros(2,5) -eye(2)], and
%     beq = [ -h m*(gVec-dJcm*dq)]'
%

% assign equality constraint terms
QP.Aeq = [       QP.Dyn.M      -eye(5)    zeros(5,2); ...
          QP.Dyn.m*QP.Kin.Jcm zeros(2,5)   -eye(2)  ];

QP.beq = [           -QP.Dyn.h; ...
          QP.Dyn.m*(QP.Dyn.gVec-QP.Kin.dJcmxdq)];

      
% Theory:
% (1) The horizontal friction needs to stay within the friction cone,
%     Fx <= mu*Fy, where mu is the friction coefficient.
%
% (2) The equation can be reformulated into 
%     [1 -mu]*[Fx Fy]' <= 0
%
% (2) The corresponding  constraint is given by
%     Aineq*x <= bineq, with
%     x   = [ddq tau F]', 
%     Aineq = [zeros(1,10) 1 -mu], and
%     bineq = 0
%
% (3) Similarly, Fx >= -mu*Fy, which can be formulated as
%     [-1 -mu]*[fX Fy]' <= 0, is implemented as inequality
%     constraint on [ddq tau F]'
      
 QP.Aineq = []
 QP.bineq = [];


     