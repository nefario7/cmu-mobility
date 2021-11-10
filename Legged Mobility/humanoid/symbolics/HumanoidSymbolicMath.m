
%
% HumanoidSymbolicMath.m  -  Symbolic math computations of the humanoid dynamics.
%                            The resulting symbolic expression are converted into
%                            a Matlab function in the script ConvertSymbolicToFunctions.m
%                            The function takes inputs q and dq and outputs kinematic and 
%                            dynamics qunatities (M, h, Jcm, Jfp, dJ*dq) needed in the
%                            main program, for instance, in the QP constraint and cost sections.
%
% H. Geyer, Nov 2018
%

% 
% Overall definitions:
% --------------------
%
% All segment angles are defined in world frame, measuring segment rotations
% from the horziontal axis and in counterclockwise directions.  
%
% The world frame origin coincides with the foot point of the right leg.
% The right leg is composed of shank (1st segment) and thigh (2nd segment)
% The left leg is composed of shank (5th segment) and thigh (4th segment)
% The two legs are connected to the trunk (3rd segment).
%
%                |
%                | d3,l3,m3,I3
%                |
%             q4 o q3
%               / \
%  d4,l4,m4,I4 /   \ d2,l2,m2,I2
%             /     \
%      ----- o q5    o q2
% d5,l5,m5,I5       /           
%             y ^  / d1,l1,m1,I1
%               | /
%              --o q1
%                |-> x

clc

% define joint and rigid body symbols
syms q1   q2   q3   q4   q5   ...  % joint angles
     dq1  dq2  dq3  dq4  dq5  ...  % joint angular velocities
     l1   l2   l3   l4   l5   ...  % segment lengths
     d1   d2   d3   d4   d5   ...  % distance to segment CoM measured from base
     m1   m2   m3   m4   m5   ...  % segment masses
     I1   I2   I3   I4   I5   ...  % segment inertias
     
% collect in group symbols for indexed processing 
q   = [  q1;   q2;   q3;   q4;   q5];
dq  = [ dq1;  dq2;  dq3;  dq4;  dq5];
l   = [  l1;   l2;   l3;   l4;   l5];
d   = [  d1;   d2;   d3;   d4;   d5];
m   = [  m1;   m2;   m3;   m4;   m5];
I   = [  I1;   I2;   I3;   I4;   I5];

assume([q dq l d m I],'real')

% joint vector group
rJ = sym(zeros(2,5)); 
rJ(:,1) = sym([0;0]);  
rJ(:,2) = rJ(:,1) +l(1)*[cos(q(1)); sin(q(1))]; 
rJ(:,3) = rJ(:,2) +l(2)*[cos(q(2)); sin(q(2))];
rJ(:,4) = rJ(:,3);  
rJ(:,5) = rJ(:,4) +l(4)*[cos(q(4)); sin(q(4))];

% segment CoM vector group
rCMs = sym(zeros(2,5)); 
for sx=1:5
    rCMs(:,sx) = rJ(:,sx) + d(sx)*[cos(q(sx)); sin(q(sx))];
end


%% Humanoid Center of Mass (rCM), related Jacobian (Jcm) and its Derivative multiplied by Joint Velocities (dJcmxdq)
% ------------------------------------------------------------------------------------------------------------------

% center of mass vector
rCM = rCMs*m / sum(m);

% center of mass Jacobian
Jcm = simplify( jacobian(rCM, q) );

% center of mass velocity
drCM = simplify(Jcm*dq);

% term dJ*dq
[nRow, nCol] = size(Jcm);
dJcm = sym(zeros(nRow,nCol));
for rx=1:nRow
    for cx=1:nCol
        for sx=1:5
            dJcm(rx,cx) = dJcm(rx,cx) +diff(Jcm(rx,cx),q(sx))*dq(sx);
        end
    end
end
dJcmxdq = simplify(dJcm*dq);


%% Swing Foot Point (rFP), related Jacobian (Jfp) and its Derivative multiplied by Joint Velocities (dJfpxdq)
% -----------------------------------------------------------------------------------------------------------

% swing foot vector
rFP = rJ(:,5) + l(5)*[cos(q(5)); sin(q(5))];

% swing foot Jacobian
Jfp = simplify( jacobian(rFP, q) ); 

% term dJ*dq
[nRow, nCol] = size(Jfp);
dJfp = sym(zeros(nRow,nCol));
for rx=1:nRow
    for cx=1:nCol
        for sx=1:5
            dJfp(rx,cx) = dJfp(rx,cx) + diff( Jfp(rx,cx), q(sx)) * dq(sx);
        end
    end
end
dJfpxdq = simplify(dJfp*dq);


%% Components of the Manipulator Equations of Motion:
% ---------------------------------------------------
%
% Theory:
% The equations of motion are given in matrix form by
% M(q)*ddq + C(q,dq)*dq + N(q) = tau,
% where M is the manipulator mass matrix, 
%       C is the Coriolis matrix, and
%       N is the vector of gravitational terms
%
% The equation is often summarized as
% M(q)*ddq +h(q,dq) = tau with h(q,dq) = C(q,dq)*dq+N(q)


%% Mass Matrix (M)
%
% Theory: 
% M = Sum_i (Ji^T*Mi*Ji), where 
% Mi is the generalized inertia matrix of body i, and
% Ji is the corresponding body Jacobian

% generalized inertia matrices
M1 = diag([m1 m1 I1]);
M2 = diag([m2 m2 I2]);
M3 = diag([m3 m3 I3]);
M4 = diag([m4 m4 I4]);
M5 = diag([m5 m5 I5]);

% body Jacobians (for translational and rotational degrees of freedom)
J1 = jacobian(rCMs(:,1),q); J1 = [J1; 1 0 0 0 0];
J2 = jacobian(rCMs(:,2),q); J2 = [J2; 0 1 0 0 0];
J3 = jacobian(rCMs(:,3),q); J3 = [J3; 0 0 1 0 0];
J4 = jacobian(rCMs(:,4),q); J4 = [J4; 0 0 0 1 0];
J5 = jacobian(rCMs(:,5),q); J5 = [J5; 0 0 0 0 1];

% humanoid mass matrix
M = J1'*M1*J1 + J2'*M2*J2 + J3'*M3*J3 + J4'*M4*J4 + J5'*M5*J5;
M = simplify(M);


%% Coriolis Matrix (C)
%
% Theory: 
% The Coriolis matrix C is a direct derivative of the mass matrix M.
% Elementwise, C is computed as
% Cij(q,dq) = 1/2*Sum_k ( partial(Mij, qk) + partial(Mik, qj) + partial(Mkj, qi) * dqk 
% where 'partial' refers to the partial derivative of the first input with respect to the second one.

% direct computation of C via its elements
[nRow, nCol] = size(M);
C = sym( zeros(nRow,nCol) );
for rx=1:nRow
    for cx=1:nCol
        for sx=1:5
           C(rx,cx) = C(rx,cx) + ( diff(M(rx,cx),q(sx)) + diff(M(rx,sx),q(cx)) - diff(M(sx,cx),q(rx)) ) * dq(sx);
        end          
    end
end
C = simplify( 1/2*C );


%% Vector of Gravity Terms (N)
% 
% Theory:
% The potential energy of the manipulator is defined as
% V(q) = Sum_i Vi(q) = Sum_i (mi*g*yi(q)), where mi is the mass of body i,
% and yi(q) is the height of this body's center of mass.
%
% Given the potential energy V(q), N(q) is given by
% N(q) = gradient(V(q)),q)

% potential energy of manipulator
V = rCMs(2,:)*m;

% gravity terms vector N
N = simplify( gradient(V,q) );


%% Summary Vector (h)
h = simplify(C*dq + N);




