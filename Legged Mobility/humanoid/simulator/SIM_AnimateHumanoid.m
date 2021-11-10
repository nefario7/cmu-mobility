function SIM_AnimateHumanoid(HMD, q, FigNr)

%
% SIM_AnimateHumanoid.m  -- Animate humanoid model
%
% Input:
% q:         joint angles in world frame coordinates
% HMDparams: humanoid model parameters
%
% Output:
% none
%

% ******* %
% PROGRAM %
% ******* %

s = sin(q); c=cos(q);


%% Update Animation Plot
% ----------------------

figure(FigNr)

HMD.Anim.RLeg.XData   = [0,  HMD.params.l1*c(1), HMD.params.l1*c(1)+HMD.params.l2*c(2)];
HMD.Anim.RLeg.YData   = [0,  HMD.params.l1*s(1), HMD.params.l1*s(1)+HMD.params.l2*s(2)];
HMD.Anim.LLeg.XData   = HMD.Anim.RLeg.XData(end) + [0,  HMD.params.l4*c(4),  HMD.params.l4*c(4)+HMD.params.l5*c(5)];
HMD.Anim.LLeg.YData   = HMD.Anim.RLeg.YData(end) + [0,  HMD.params.l4*s(4),  HMD.params.l4*s(4)+HMD.params.l5*s(5)];
HMD.Anim.Trunk.XData  = HMD.Anim.RLeg.XData(end) + [0,  HMD.params.l3*c(3)];
HMD.Anim.Trunk.YData  = HMD.Anim.RLeg.YData(end) + [0,  HMD.params.l3*s(3)];

% show CoM
CM_pos = SIM_ComputeHumanoidKinematics(q, [0 0 0 0 0]');  % CoM Jacobian and related term dJ*dq   
HMD.Anim.CM.XData = CM_pos(1);  HMD.Anim.CM.YData = CM_pos(2);

% force updating of plot
drawnow






