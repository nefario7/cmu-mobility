
FigNr = 3;
nt=length(HMD.CM_a(1,:));


figure(FigNr), clf


%% Joint Accelerations
% --------------------
subplot(241), hold on
plot(1000*tVec, 180/pi*HMD.q', '-x', 'LineWidth', 2)
xlabel('time (ms)'),  ylabel('q_i (deg)'),  title('Joint Angles')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5')


%% Joint Torques
% --------------
subplot(245), hold on
plot(1000*tVec, HMD.tau', '-x', 'LineWidth', 2)
xlabel('time (ms)'),  ylabel('\tau_i (Nm)'),  title('Joint Torques')
legend('\tau_1', '\tau_2', '\tau_3', '\tau_4', '\tau_5')


%% CoM Position
% -------------
subplot(442),  hold on, 
plot(1000*tVec(1:nt), HMD.CM_p(1,1:nt)', 'r-o', 'LineWidth', 3)
plot(1000*tVec(1:nt), CM_pdes(1,1:nt),'k--','LineWidth',2)
xlabel('time (ms)'),  ylabel('CoM position (m)'),   title('Huamnoid CoM')
legend('CM_{x,HMD}', 'CM_{x,MPC}')
subplot(446),  hold on, 
plot(1000*tVec(1:nt), HMD.CM_p(2,1:nt)', 'r-o', 'LineWidth', 3)
plot(1000*tVec(1:nt), CM_pdes(2,1:nt),'k--','LineWidth',2)
xlabel('time (ms)'),  ylabel('CoM position (m)')
legend('CM_{y,HMD}', 'CM_{y,MPC}')


%% GRFs
% -----
subplot(4,4,10), hold on
plot(1000*tVec(1:nt), HMD.GRF(1,1:nt)', 'r-o', 'LineWidth', 3)
plot(1000*tVec(1:nt), QP.GRF(1,1:nt)', 'k--', 'LineWidth', 2)
xlabel('time (ms)'),  ylabel('GRF_x (N)'),  title('Ground Reaction Forces')
legend('GRF_{x}', 'GRF_{x,QP}')
subplot(4,4,14), hold on
plot(1000*tVec(1:nt), HMD.GRF(2,1:nt)', 'r-o', 'LineWidth', 3)
plot(1000*tVec(1:nt), QP.GRF(2,1:nt)', 'k--', 'LineWidth', 2)
xlabel('time (ms)'),  ylabel('GRF_y (N)')
legend('GRF_{y}', 'GRF_{y,QP}')


%% CoM Cost
% ---------
subplot(243), hold on
plot(1000*tVec(1:nt),HMD.CM_a(1,1:nt),'r-o','LineWidth',3)
plot(1000*tVec(1:nt),HMD.CM_a(2,1:nt),'-o','Color',[0.5 0.5 1], 'LineWidth',3)
plot(1000*tVec(1:nt),CM_ades(1,1:nt),'k--','LineWidth',2)
plot(1000*tVec(1:nt),CM_ades(2,1:nt),'--','Color',[0.25 0.25 0.25], 'LineWidth',2)
xlabel('time (ms)'),  ylabel('acceleration (m/s^2)'), title('Desired vs. Achieved CoM Accelerations')
legend('CM_{a,x}', 'CM_{a,y}', 'CM_{ades,x}', 'CM_{ades,y}')


%% Trunk Cost
% -----------
subplot(244), hold on
plot(1000*tVec(1:nt),180/pi*HMD.ddq(3,1:nt),'r-o','LineWidth',3)
plot(1000*tVec(1:nt),180/pi*TR_ades(1:nt),'k--','LineWidth',2)
xlabel('time (ms)'),  ylabel('acceleration (deg/s^2)'), title('Desired vs. Achieved Trunk Accelerations')
legend('TR_{a}', 'TR_{ades}')


%% Foot Point Cost
% ----------------
subplot(247), hold on
plot(1000*tVec(1:nt),HMD.FP_a(1,:),'r-o','LineWidth',3)
plot(1000*tVec(1:nt),HMD.FP_a(2,:),' -o','Color',[0.5 0.5 1],'LineWidth',3)
plot(1000*tVec(1:nt),FP_ades(1,1:nt),'k--','LineWidth',2)
plot(1000*tVec(1:nt),FP_ades(2,1:nt),'--','Color',[0.25 0.25 0.25], 'LineWidth',2)
xlabel('time (ms)'),  ylabel('acceleration (m/s^2)'), title('Desired vs. Achieved FP Accelerations')
legend('FP_{a,x}', 'FP_{a,y}', 'FP_{ades,x}', 'FP_{ades,y}')





