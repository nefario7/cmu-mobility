function MPC = MPC_ConstraintMatrices(MPC)

%
% [Aineq, G1, G2, G3, uMinVec, uMaxVec] = CoonstraintMatrices(MPC)
%                   Compute the constant constraint matrices of the cost function
%                   that corresponds to the Constrained LTI MPC problem.
%
% Input:  MPC object (compare function "InitModelMPC.m"
% Output: LTI constraint function terms Aineq, G1, G2 and G3 where 
%         Aineq*u <= Bineq with Bineq = G1*x(k) + G2*u(k-1) +G3
%
%         as well as direct saturation constraint vectors uMinVec and
%         uMaxVec. All these terms are added to the MPC object.
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

% constrained output matrix
Cc = MPC.Cc;

% constrained output direct term
Dc = MPC.Dc;



% -------------------
% Assign Matrix Sizes
% -------------------

% lengths of state and input and constrained output vectors
nx = MPC.nx;
nu = MPC.nu;
nc = MPC.nc;






% ---------------------------------------------------------------
% Iteratively Build Constraint Function Matrices Aineq, G1 and G3
% ---------------------------------------------------------------


% initialize the functions 
% ------------------------

% Aineq
Aineq_1top = []; Aineq_1bottom = []; 

% G1 components
G1_1top = []; G1_1bottom = [];

% G3 components
G3_1top = []; G3_1bottom = []; G3_2top = []; G3_2bottom = [];


% initialize functions phi and psi (page 40, M. Alamir book) 
Phi_i = A;
Psi_i = B;

% loop trough prediction horizon with index phIdx
for phIdx = 1:N
    
  % compose current selection matrices for input and output
  S_u = MPC_SelectionMatrix(phIdx, nu, N);
  
  % pad current function psi with zeros
  Psi_i_padded = [Psi_i zeros(nx, (N-phIdx)*nu)];
  
  
  % First set of constraint equations (page 51, Alamir)
  % ---------------------------------------------------
  
  % Aineq_1
  Aineq_1top    = [   Aineq_1top;  (Cc*Psi_i_padded +Dc*S_u)];
  Aineq_1bottom = [Aineq_1bottom; -(Cc*Psi_i_padded +Dc*S_u)];
  
  % G1_1
  G1_1top       = [      G1_1top;                -Cc*Phi_i];
  G1_1bottom    = [   G1_1bottom;                 Cc*Phi_i];
  
  % G3_1
  G3_1top       = [      G3_1top;                MPC.ycMax];
  G3_1bottom    = [   G3_1bottom;               -MPC.ycMin];
  
  
  % Second set of constraint equations (page 52, Alamir)
  % ---------------------------------------------------
  
  % iteratively compose G3_2 (page 52)
  G3_2top    = [         G3_2top;                MPC.duMax];
  G3_2bottom = [      G3_2bottom;               -MPC.duMin];
  
  % iterate functions phi and psi
  Phi_i = Phi_i*A;
  Psi_i = [A*Psi_i B];
  
end % for phIdx = 1:N 



% ----------------------------------------------
% Assemble matrices of first set of inequalities
% ----------------------------------------------

Aineq_1 = [Aineq_1top; Aineq_1bottom];
G1_1    = [   G1_1top;    G1_1bottom];
G2_1    = zeros(2*N*nc, nu);
G3_1    = [   G3_1top;    G3_1bottom];



% ----------------------
% Build Aineq_2 and G2_2
% ----------------------

% initialize Aineq_2top
Aineq_2top=eye(N*nu);

% compose Aineq_2top 
for phIdx=2:N
    rowidc = ((phIdx-1)*nu+1) : phIdx*nu;
    colidc = ((phIdx-2)*nu+1) : (phIdx-1)*nu;
    Aineq_2top(rowidc,colidc) = -eye(nu);
end

% G2_2top
G2_2top = [eye(nu); zeros((N-1)*nu,nu)];



% -----------------------------------------------
% Assemble matrices of second set of inequalities
% -----------------------------------------------

Aineq_2 = [Aineq_2top; -Aineq_2top];
G1_2    = zeros(2*N*nu, nx);
G2_2    = [   G2_2top;    -G2_2top];
G3_2    = [   G3_2top;  G3_2bottom];



% ---------------------------------
% Assemble full inequality matrices
% ---------------------------------

MPC.Aineq = [Aineq_1; Aineq_2];
MPC.G1    = [   G1_1;    G1_2];
MPC.G2    = [   G2_1;    G2_2];
MPC.G3    = [   G3_1;    G3_2];



% --------------------------
% Compute bounds on u vector
% --------------------------

MPC.uMaxVec = ones(N,1)*MPC.uMax;
MPC.uMinVec = ones(N,1)*MPC.uMin;












