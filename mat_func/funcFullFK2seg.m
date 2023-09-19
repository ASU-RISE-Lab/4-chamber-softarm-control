function [] = funcFullFK2seg(testData,par_set)
mocapResult=[];
mocapResult = funcComputeStateVar_v2(testData,par_set);
fkResult = funcCompuFK2seg_v1(mocapResult.state_array_wire);
close all
figure(1)
subplot(3,1,1)
plot(fkResult.camFrameE1(:,1),'r')
hold on
plot(fkResult.camFrameE1(:,2),'b')
hold on
plot(fkResult.camFrameE1(:,3),'k')
hold on

plot(testData.rigid_2_pose(:,1),'r--')
hold on
plot(testData.rigid_2_pose(:,2),'b--')
hold on
plot(testData.rigid_2_pose(:,3),'k--')
hold on
title('FK result for Endeffector 1 in Cam frame')
legend('xfk','yfk','zfk','xmo','ymo','zmo')
subplot(3,1,2)

plot((mocapResult.state_array(:,1)-min(mocapResult.state_array(:,1)))./(max(mocapResult.state_array(:,1))-min(mocapResult.state_array(:,1))))
hold on
plot((mocapResult.u_pm_tf(:,1)-min(mocapResult.u_pm_tf(:,1)))./(max(mocapResult.u_pm_tf(:,1))-min(mocapResult.u_pm_tf(:,1))))
hold on
plot((mocapResult.state_array(:,3)-min(mocapResult.state_array(:,3)))./(max(mocapResult.state_array(:,3))-min(mocapResult.state_array(:,3))))
hold on
plot((mocapResult.u_pm_tf(:,2)-min(mocapResult.u_pm_tf(:,2)))./(max(mocapResult.u_pm_tf(:,2))-min(mocapResult.u_pm_tf(:,2))))
hold on
legend('theta1','tau1','lc1','f1')
title('Normalized input-output pair')
subplot(3,1,3)

plot(fkResult.camFrameE1(:,1)-testData.rigid_2_pose(:,1))
hold on
plot(fkResult.camFrameE1(:,2)-testData.rigid_2_pose(:,2))
hold on
plot(fkResult.camFrameE1(:,3)-testData.rigid_2_pose(:,3))
hold on
title('fk errors in cam frame')
legend('x','y','z')

figure(2)
subplot(3,1,1)
plot(fkResult.camFrameE2(:,1),'r')
hold on
plot(fkResult.camFrameE2(:,2),'b')
hold on
plot(fkResult.camFrameE2(:,3),'k')
hold on

plot(testData.rigid_3_pose(:,1),'r--')
hold on
plot(testData.rigid_3_pose(:,2),'b--')
hold on
plot(testData.rigid_3_pose(:,3),'k--')
hold on
title('FK result for Endeffector 2 in Cam frame')
legend('xfk','yfk','zfk','xmo','ymo','zmo')
subplot(3,1,2)

plot((mocapResult.state_array(:,5)-min(mocapResult.state_array(:,5)))./(max(mocapResult.state_array(:,5))-min(mocapResult.state_array(:,5))))
hold on
plot((mocapResult.u_pm_tf(:,3)-min(mocapResult.u_pm_tf(:,3)))./(max(mocapResult.u_pm_tf(:,3))-min(mocapResult.u_pm_tf(:,3))))
hold on
plot((mocapResult.state_array(:,7)-min(mocapResult.state_array(:,7)))./(max(mocapResult.state_array(:,7))-min(mocapResult.state_array(:,7))))
hold on
plot((mocapResult.u_pm_tf(:,4)-min(mocapResult.u_pm_tf(:,4)))./(max(mocapResult.u_pm_tf(:,4))-min(mocapResult.u_pm_tf(:,4))))
legend('theta1','tau1','lc1','f1')
title('Normalized input-output pair')
subplot(3,1,3)
plot(fkResult.camFrameE2(:,1)-testData.rigid_3_pose(:,1))
hold on
plot(fkResult.camFrameE2(:,2)-testData.rigid_3_pose(:,2))
hold on
plot(fkResult.camFrameE2(:,3)-testData.rigid_3_pose(:,3))
hold on
title('fk errors in cam frame')
legend('x','y','z')
end

