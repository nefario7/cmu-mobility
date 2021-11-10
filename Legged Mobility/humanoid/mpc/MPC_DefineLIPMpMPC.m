function MPC = MPC_DefineLIPMpMPC(mParams, Qpos, Qvel, R, tR, xLIPM,dxLIPM)
%
% MPC = DEFINEMPC - Define MPC Problem including dynamics 
%                   and cost function parameters
%
% Output: Object 'MPC' containing all specific information for a
%         LTI MPC problem.
%
% MPC.A: discrete time dynamics matrix
% MPC.B: discrete time input matrix
% 
% MPC.dt: sampling period
% MPC.N:  horizon length in samples
%
% MPC.Cr: output regulation matrix
% MPC.Qy: regulation output weighing matrix
% MPC.Qu: input weight
%
% H Geyer, Nov 2018
%

% create or clear MPC object 
MPC = [];

% prediction horizon
MPC.dt = mParams.dt;            %[s] sampling period
MPC.N = floor(tR/MPC.dt); %[# of samples] horizon length

% constraints
MPC.Cc = [1 0];                                 % constrained output matrix
MPC.nc = size(MPC.Cc,1);                        % number of elements in constrained output matrix
MPC.ycMin = -sqrt(mParams.lMax^2-mParams.y0^2); % lower output limit
MPC.ycMax =  sqrt(mParams.lMax^2-mParams.y0^2); % upper output limit
MPC.uMin  = mParams.pMin;                       % lower input limit 
MPC.uMax  = mParams.pMax;                       % upper input limit
MPC.duMin = -abs(mParams.pMax-mParams.pMin);    % lower input rate limit [per sample]
MPC.duMax =  abs(mParams.pMax-mParams.pMin);    % upper input rate limit [per sample]

% cost function
MPC.Cr = [1 0; 0 1];        % regulated output matrix
MPC.ny = size(MPC.Cr,1);    % number of regulated outputs
MPC.Qy = diag([Qpos Qvel]); %diag([Qpos Qvel]); % regulated output weighing matrix
MPC.Qu = R;                 % control input weight

% initialize input value from previous step, u(k-1) 
MPC.uPrev = 0; 

% compute discrete time dynamics
w = sqrt(mParams.g/mParams.y0);         % eigenfrequency of LIPM
MPC.A = [1 MPC.dt; w^2*MPC.dt 1]; % discrete time dynamics matrix
MPC.B = [0; -w^2*MPC.dt];         % discrete time control matrix

% assign dependent elements of MPC                                        
[MPC.nx, MPC.nu] = size(MPC.B);                % number of elements in state and inpout vector
MPC.Dc = zeros(size(MPC.Cc,1), size(MPC.B,2)); % constrained output direct term

% create reference output vector at update rate of MPC
tVec=(0:MPC.dt:tR)'; nt=length(tVec);                  % create sample time vector
xLIPMinterp  = interp1(0:mParams.dt:tR,  xLIPM, tVec); % interpolate  xLIPM to fit MPC sampling rate
dxLIPMinterp = interp1(0:mParams.dt:tR, dxLIPM, tVec); % interpolate dxLIPM to fit MPC sampling rate
MPC.yRef = NaN(nt*2,1);                                % create empty reference vector
MPC.yRef(1:2:(nt*2-1)) = xLIPMinterp;                  % assemble reference output: positions
MPC.yRef(2:2:nt*2) = dxLIPMinterp;                     % assemble reference output: velocities

% create and initialize state, output and control vector
MPC.xVec = NaN(nt, MPC.nx);  MPC.yVec = NaN(nt, MPC.ny); MPC.uVec = NaN(nt, MPC.nu);
x0 = [xLIPM(1) dxLIPM(1)];
MPC.xVec(1,:) = x0;  MPC.yVec(1,:) = (MPC.Cr*x0')';

% set matlab quadprog search parameters
MPC.options = optimoptions('quadprog','Display','off', 'Diagnostics','off','MaxIterations',200);

























