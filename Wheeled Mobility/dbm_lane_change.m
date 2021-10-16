% Lane Change Task
step = 0.01;

% Inputs for Desired Lane Change (psi_dot) 
total_steps = floor(5/(vx * step)) + floor(90/(vx * step)) + floor((5 + vx * 1)/(vx * step)) + 2;
p1 = zeros(1, floor(5/(vx * step)) + 1);
p2 = ones(1, floor(90/(vx * step)) + 1) * atan(5/90);
p3 = zeros(1, floor((5 + vx * 1)/(vx * step)));
psi_lane = [p1 p2 p3 0];
psi_dot_lane = diff(psi_lane);

% LQR Controller
Q = C' * C;
R = 10;
Q(1,1) = 1;
Q(3,3) = 3;
K = lqr(A, B1, Q, R);

% System Dynamics 
Ac = (A - B1 * K);
Bc = B2;
Cc = C;
Dc = 0;

% Closed Loop System
closed_system_lane = ss(Ac, Bc, Cc, Dc);

% Response and inputs
tin = 0:step:(total_steps - 1) * step;
[r, tout, sv] = lsim(closed_system_lane, psi_dot_lane, tin);
e1 = sv(:, 1);
e2 = sv(:, 3);

delta = ones(1, length(sv));
for i = 1:length(sv)
    delta(i) = -K * transpose(sv(i, :));
end
delta_dot = diff([delta 0]);
delta_dot = delta_dot / step;

% Plots
% Steering Rate
figure()
plot(tout, delta_dot)
title('Rate of Steering')
xlabel("Time, sec")
ylabel("Steering Rate, rad/s")

% Errors
figure()
subplot(1, 2, 1)
plot(tout, e1)
title("Lateral Position Error")
xlabel("Time, sec")
ylabel("Lateral Position Error, e1")
subplot(1, 2, 2)
plot(tout, e2)
title("Yaw Angle Error")
xlabel("Time, sec")
ylabel("Yaw Angle Error, e2")


% Specifications and Constraints
t1 = floor(5/(vx * step)):floor(5/(vx * step))+ 1/step;
t2 = floor(90/(vx * step)):floor(90/(vx * step)) + 1/step;
if max(abs(delta_dot)) >= 25.0
    fprintf("Steering exceeded %f!", max(abs(delta_dot)))
end
if max(abs(e1)) > 0.01 
    fprintf("Max e1 Condition NOT Satisfied! (%f)\n", max(abs(e1)))
end
if max(abs(e1(t1))) > 0.002 && max(abs(e2(t1))) > 0.0007
    fprintf("Transition 1 Conditions NOT Satisfied! (%f, %f)\n", max(abs(e1(t1))), max(abs(e2(t1))))
end

if max(abs(e1(t2))) > 0.002 && max(abs(e2(t2))) > 0.0007
    fprintf("Transition 2 Conditions NOT Satisfied! (%f, %f)\n", max(abs(e1(t2))), max(abs(e2(t2))))
end


% Desired Trajectory
dx = zeros(1, total_steps);
dy = zeros(1, total_steps);
dx(1) = 0;
dy(1) = -5;
for k=2:total_steps
    dx(k) = dx(k-1) + vx * step * cos(psi_lane(k));
    dy(k) = dy(k-1) + vx * step * sin(psi_lane(k));
end
figure()
plot(dx, dy, '--')
hold on

% Lane Change
wx = zeros(1, total_steps);
wy = zeros(1, total_steps);
wx(1) = 0;
wy(1) = -5;
for i = 2:total_steps
    wpsi = psi_lane(i) + e2(i);
    wx(i) = wx(i-1) + vx * step * cos(wpsi) + e1(i) * sin(wpsi);
    wy(i) = wy(i-1) + vx * step * sin(wpsi) + e1(i) * cos(wpsi);
end
plot(wx, wy)
title("Desired Trajectory - Lane Change")
legend('Desired','Actual')
xlabel("X")
ylabel("Y")