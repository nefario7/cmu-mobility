function MPC = MPC_CostMatrices(MPC)

%
% [H, F1, F2, F3] = CostMatrices(MPC)
%                   Compute the constant cost matrices of the cost function
%                   that corresponds to the LTI MPC problem.
%
% Input:  MPC object (compare function "InitModelMPC.m"
% Output: LTI cost function terms H, F1, F2 and F3 where cost fct
%         J = 1/2*u^T*H*u + (F1*x+F2*y_desired+F3*u_desired)^T*u + const.
%         All terms are stored in the MPC object.
%
% H Geyer, Jun 2014
%



% ----------------
% Assign Variables
% ----------------

% dynamics and input matrices
A = MPC.A;
B = MPC.B;

% prediction horizon
N =MPC.N;

% regulated output and weiging matrices related to cost function J
Cr = MPC.Cr;
Qy = MPC.Qy;
Qu = MPC.Qu;



% -------------------
% Assign Matrix Sizes
% -------------------

% lengths of state and imput vector
nx = MPC.nx;
nu = MPC.nu;

% length of regulated output
ny = size(Cr,1);

% Hessian
nH = N*nu;



% ------------------
% Preallocate Memory 
% ------------------

% Hessian
H  = zeros(nH, nH);

% F1 through F3
F1 = zeros(nH, nx);
F2 = zeros(nH, N*ny);
F3 = zeros(nH, nu);



% ----------------------------------------
% Iteratively Build Cost Function Matrices
% ----------------------------------------

% initialize functions phi and psi (page 40, M. Alamir book) 
Phi = A;
Psi = B;

% loop trough prediction horizon with index phIdx
for phIdx = 1:N
    
  % compose current selection matrices for input and output
  S_u = MPC_SelectionMatrix(phIdx, nu, N);
  S_y = MPC_SelectionMatrix(phIdx, ny, N);
  
  % pad current function psi with zeros
  Psi_padded = [Psi zeros(nx, (N-phIdx)*nu)];
  
  % compute current H (page 44, Alamir)
  H =  H +Psi_padded' * Cr' * Qy * Cr * Psi_padded  +S_u' * Qu * S_u;
  
  % compute current F1 through F3 (page 44, Alamir)
  F1 = F1 +Psi_padded' * Cr' * Qy * Cr * Phi;
  F2 = F2 -Psi_padded' * Cr' * Qy * S_y;
  F3 = F3 +S_u'*Qu;
  
  % iterate functions phi and psi
  Phi = Phi*A;
  Psi = [A*Psi B];
  
end % for phIdx = 1:N 

% finalize matrices
MPC.H  = 2*H;
MPC.H  = (MPC.H+MPC.H')/2; % remove assymmetry due to roumd-off errors
MPC.F1 = 2*F1;
MPC.F2 = 2*F2;
MPC.F3 = 2*F3;












