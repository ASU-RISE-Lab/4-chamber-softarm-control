function []=func_compare_kinematic(tip_exp,xyz_estimation,par)
figure
subplot(3,1,1)
plot(tip_exp(:,2),'Color','r','LineWidth',2,'LineStyle','-.')
hold on
plot(xyz_estimation(:,1),'Color','b','LineWidth',2)
title('Tip Position(m) on X-Axis')
legend('Experiment','Model')
ylim([-0.05,0.2])
subplot(3,1,2)
plot(tip_exp(:,3),'Color','r','LineWidth',2,'LineStyle','-.')
hold on
plot(xyz_estimation(:,2),'Color','b','LineWidth',2)
title('Tip Position(m) on Y-Axis')
ylim([-0.05,0.2])
subplot(3,1,3)
plot(-par.L+tip_exp(:,4),'Color','r','LineWidth',2,'LineStyle','-.')
hold on
plot(-par.L+xyz_estimation(:,3),'Color','b','LineWidth',2)
title('Tip Position(m) on Z-Axis')
ylim([-0.2,0.05])

error_matrix=tip_exp(:,2:4)-xyz_estimation;
RMSE_x = sqrt(mean((error_matrix(:,1)).^2))
RMSE_y = sqrt(mean((error_matrix(:,2)).^2))
RMSE_z = sqrt(mean((error_matrix(:,3)).^2))

figure
subplot(3,1,1)
plot((tip_exp(:,2)-xyz_estimation(:,1))./par.L,'Color','r','LineWidth',2,'LineStyle','-.')
title('Normalized Error on X-Axis')
% legend('Experiment','Model')
% ylim([-0.2,0.2])
subplot(3,1,2)
plot((tip_exp(:,3)-xyz_estimation(:,2))./par.L,'Color','r','LineWidth',2,'LineStyle','-.')
title('Normalized Error on Y-Axis')
% legend('Experiment','Model')
% ylim([-0.2,0.2])
subplot(3,1,3)
plot((tip_exp(:,4)-xyz_estimation(:,3))./par.L,'Color','r','LineWidth',2,'LineStyle','-.')
title('Normalized Error on Z-Axis')
% legend('Experiment','Model')
% ylim([-0.2,0.2])
end