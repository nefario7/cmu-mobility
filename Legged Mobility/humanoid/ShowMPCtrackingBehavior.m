function ShowMPCtrackingBehavior(MPC, LIPMparams, tVec, FigNr)

%
% ShowMPCtrackingBehavior.m  -  Show outcome of MPC tracking of nominal
%                               CoM plan.
%
% 

% initialize figure
figure(FigNr), clf

% position tracking
subplot(311), hold on
xlabel('stance time (ms)'), ylabel('CoM position x (m)')
title('\bfLIPMp Tracking of Nominal LIPM Plan') 
patch(1000*[tVec(1) tVec(end) tVec(end) tVec(1) tVec(1)], [LIPMparams.pMin LIPMparams.pMin LIPMparams.pMax LIPMparams.pMax LIPMparams.pMin], [0.8 0.8 0.8], 'EdgeColor', 'none')
plot(1000*tVec, MPC.yRef(1:2:length(MPC.yRef)-1), 'k-x','LineWidth',2), 
plot(1000*tVec, MPC.yVec(:,1), 'r-o','LineWidth',2)
plot(1000*[tVec(1) tVec(end)], [1 1]*MPC.ycMin, 'k--')
plot(1000*[tVec(1) tVec(end)], [1 1]*MPC.ycMax, 'k--')
ylim(1.1*[MPC.ycMin MPC.ycMax])
legend('Polygon of Support','Nominal LIPM path','MPC tracking with LIPMp','max leg length constraint','Location','NW')

% velocity
subplot(312), hold on
xlabel('stance time (ms)'), ylabel('CoM velocity (m/s)')
title('\bfLIPMp Velocity vs  LIPM Plan') 
plot(1000*tVec, MPC.yRef(2:2:length(MPC.yRef)), 'k-x','LineWidth',2), 
plot(1000*tVec, MPC.yVec(:,2), 'r-o','LineWidth',2)
legend('Nominal LIPM velocity','MPC tracking with LIPMp','Location','NW')

subplot(313), hold on
xlabel('stance time (ms)'), ylabel('CoP')
title('\bfCoP Control Input')
plot(1000*tVec, MPC.uVec, 'r-o','LineWidth',2)
plot(1000*[tVec(1) tVec(end)], [1 1]*MPC.uMin, 'k--')
plot(1000*[tVec(1) tVec(end)], [1 1]*MPC.uMax, 'k--')
ylim(1.1*[MPC.uMin MPC.uMax])
legend('CoP input','Polygon of support constraint','Location','NE')

