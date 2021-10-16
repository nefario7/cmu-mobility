% DBM Model Setup
% Vehicle Parameters
vx = 30; %m/s
m = 1573; %kg
Iz = 2873; %kg-m2
lf = 1.10; %m
lr = 1.58; %m
Caf = 80000; %N/rad
Car = 80000; %N/rad

% System Dynamics
A = [0 1 0 0;
     0 1 1 1;
     0 0 0 1;
     0 1 1 1];
A(2,2) = -(2 * Caf + 2 * Car)/(m * vx);
A(2,3) = (2 * Caf + 2 * Car)/(m);
A(2,4) = -(2 * Caf * lf - 2 * Car * lr)/(m * vx);
A(4,2) = -(2 * Caf * lf - 2 * Car * lr)/(Iz * vx);
A(4,3) = (2 * Caf * lf - 2 * Car * lr)/(Iz);
A(4,4) = -(2 * Caf * (lf ^ 2) - 2 * Car * (lr ^ 2))/(Iz * vx);

B1 = [0; 
      1; 
      0; 
      1];
B1(2,1) = 2 * Caf / m;
B1(4,1) = 2 * Caf * lf / Iz;
B2 = [0; 
      1; 
      0; 
      1];
B2(2,1) = -(2 * Caf * lf - 2 * Car * lr)/(m * vx) - vx;
B2(4,1) = -(2 * Caf * (lf ^ 2) + 2 * Car * (lr ^ 2))/(Iz * vx);
C = [1 0 0 0;
     0 0 1 0];
D = [0; 
     0]; 

ostates = {'p', 'p_dot', 'y', 'y_dot'};
oinputs = {'delta'};

% Open Loop System (Check for 2 poles at origin)
open_system = ss(A, B1, C, D);

% Controllability (Check if <4, then not controllable)
poles = eig(A);
co = ctrb(open_system);
controllability = rank(co);


