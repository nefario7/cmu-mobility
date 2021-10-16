% Curves Task
step = 0.01;

% Inputs for Desired Curved Path (psi_dot) 
total_steps_curve = 12 / step;
pdot1 = zeros(1, 1 / step);
pdot2 = ones(1, 5 / step) * vx / 1000;
pdot3 = zeros(1, 1 / step);
pdot4 = ones(1, 5 / step) * vx * (-1)/ 500;
psi_dot_curve = [pdot1 pdot2 pdot3 pdot4];
psi_curve = zeros(1, length(psi_dot_curve) + 1);
for i = 1:length(psi_curve)-1
    psi_curve(i+1) = psi_dot_curve(i) * step + psi_curve(i);
end


% LQR Controller
QC = C' * C;
RC = 1;
QC(1,1) = 500;
QC(3,3) = 10;
KC = lqr(A, B1, QC, RC);

% System Dynamics 
ACc = (A - B1 * KC);
BCc = B2;
CCc = C;
DCc = 0;

% Closed Loop System
closed_system_curves = ss(ACc, BCc, CCc, DCc);

% Response and inputs
tin = 0:step:(total_steps_curve - 1) * step;
[r_curve, tout, sv_curve] = lsim(closed_system_curves, psi_dot_curve, tin);
e1_curve = sv_curve(:, 1);
e2_curve = sv_curve(:, 3);

% Plots
% Errors
figure()
subplot(1, 2, 1)
plot(tout, e1_curve)
title("Lateral Position Error")
xlabel("Time, sec")
ylabel("Lateral Position Error, e1")
subplot(1, 2, 2)
plot(tout, e2_curve)
title("Yaw Angle Error")
xlabel("Time, sec")
ylabel("Yaw Angle Error, e2")


% Specifications and Constraints
if max(abs(e1_curve)) > 0.01 && max(abs(e2_curve)) > 0.01
    fprintf("Conditions NOT Satisfied (%f, %f)!\n", max(abs(e1_curve)), max(abs(e1_curve)))
end

%Desired Trajectory
dx_curve = zeros(1, total_steps_curve);
dy_curve = zeros(1, total_steps_curve);
dx_curve(1) = 0;
dy_curve(1) = -5;
for k=2:total_steps_curve
    dx_curve(k) = dx_curve(k-1) + vx * step * cos(psi_curve(k));
    dy_curve(k) = dy_curve(k-1) + vx * step * sin(psi_curve(k));
end
figure()
plot(dx_curve, dy_curve)
hold on

%Curves
wx_curve = zeros(1, total_steps_curve);
wy_curve = zeros(1, total_steps_curve);
wx_curve(1) = 0;
wy_curve(1) = -5;
for i = 2:total_steps_curve
    wpsi = psi_curve(i) + e2_curve(i);
    wx_curve(i) = wx_curve(i-1) + vx * step * cos(wpsi) + e1_curve(i) * sin(wpsi);
    wy_curve(i) = wy_curve(i-1) + vx * step * sin(wpsi) + e1_curve(i) * cos(wpsi);
end
plot(wx_curve, wy_curve)
title("Desired Trajectory - Curves")
legend('Desired','Actual')
xlabel("X")
ylabel("Y")