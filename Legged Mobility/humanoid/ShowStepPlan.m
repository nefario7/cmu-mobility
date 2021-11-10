function ShowStepPlan(q0, FP_pos, xLIPM, LIPMparams, HMD, FigNr)

% 
% ShowPlan.m -- Show step plan including CoM and Foot Point trajectories
%

% plot CoM trajectory
plot(xLIPM(1,:), LIPMparams.y0*ones(1, size(xLIPM,2)), 'k-x','LineWidth',1), 

% plot foot point trajectory and associated information
plot(FP_pos(1,:),FP_pos(2,:),'b-x', 'LineWidth',2)

% plot humanoid stick figure
SIM_AnimateHumanoid(HMD, q0, FigNr)



% additional elemenets
plot([-0.5 0.5],[0 0],'k')                                                   % ground
plot(0,0,'kx','MarkerSize',10), text(0,-0.005,'Stance foot origin')          % coordinate origin
plot(FP_pos(1,1),FP_pos(2,1),'bx','MarkerSize',10)                           % initial foot point
text(FP_pos(1,1),FP_pos(2,1)-0.005,'Initial swing foot point','Color','b')
plot(FP_pos(1,end),FP_pos(2,end),'bx','MarkerSize',10)                       % end point
text(FP_pos(1,end),FP_pos(2,end)-0.005,'Final swing foot point','Color','b')
axis([-0.5 0.5 -0.05 1.5])
grid

xlabel('x(m)'), ylabel('y(m)')
title('\bfNominal step plan for CoM and FP')
legend('Right leg', 'Left leg', 'Trunk', 'CoM', 'CoM trajectory', 'swing foot trajectory'), 
