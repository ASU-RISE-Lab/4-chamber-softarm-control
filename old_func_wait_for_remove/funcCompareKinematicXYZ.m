function []=funcCompareKinematicXYZ(trainSet,xyz_estimation,par_set)
fig_width=7/2;
fig_height=7/2;
%%% fig 1
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
subplot(2,1,1)
plot(trainSet.pd_MPa(:,1),trainSet.tip_exp(:,2),'Color','r','LineWidth',2)
hold on
plot(trainSet.pd_MPa(:,1),xyz_estimation(:,1),'Color','r','LineWidth',2,'LineStyle','-.')
% title('Tip Position(m) on X-Axis')
% legend('Experiment','Model')
ylim([-0.05,0.2])
% subplot(3,1,2)
plot(trainSet.pd_MPa(:,1),trainSet.tip_exp(:,3),'Color','b','LineWidth',2)
hold on
plot(trainSet.pd_MPa(:,1),xyz_estimation(:,2),'Color','b','LineWidth',2,'LineStyle','-.')
% title('Tip Position(m) on Y-Axis')
ylim([-0.05,0.2])
% subplot(3,1,3)
plot(trainSet.pd_MPa(:,1),trainSet.tip_exp(:,4),'Color','k','LineWidth',2)
hold on
plot(trainSet.pd_MPa(:,1),xyz_estimation(:,3),'Color','k','LineWidth',2,'LineStyle','-.')
% title('Tip Position(m) on Z-Axis')
% ylim([-0.025,0.2])
ylabel('Position (m)')
xlabel('Time (second)')

xlim([0,60])
error_matrix=trainSet.tip_exp(:,2:4)-xyz_estimation;
RMSE_x = sqrt(mean((error_matrix(:,1)).^2))
RMSE_y = sqrt(mean((error_matrix(:,2)).^2))
RMSE_z = sqrt(mean((error_matrix(:,3)).^2))
leg=legend({'$n_{exp}$','$n_m$','$o_{exp}$','$o_m$','$s_{exp}$','$s_m$'},'Orientation','vertical','Location','eastoutside','Units','inches','Interpreter','latex')
leg.ItemTokenSize = [20,20];
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
subplot(2,1,2)
plot(trainSet.pd_MPa(:,1),abs(trainSet.tip_exp(:,2)-xyz_estimation(:,1))./par_set.L,'Color','r','LineWidth',2)
% title('Normalized Error on X-Axis')
% legend('Experiment','Model')
% ylim([-0.2,0.2])
hold on
plot(trainSet.pd_MPa(:,1),abs(trainSet.tip_exp(:,3)-xyz_estimation(:,2))./par_set.L,'Color','b','LineWidth',2)
% title('Normalized Error on Y-Axis')
% legend('Experiment','Model')
% ylim([-0.2,0.2])
% subplot(3,1,3)
hold on
plot(trainSet.pd_MPa(:,1),abs(trainSet.tip_exp(:,4)-xyz_estimation(:,3))./par_set.L,'Color','k','LineWidth',2)
% title('Normalized Error on Z-Axis')
% legend('Experiment','Model')
hold on
ylabel('Normalized Error')
xlabel('Time (second)')
xlim([0,60])
ylim([0,0.03])
leg=legend({'$n_{err}$','$o_{err}$','$s_{err}$'},'Orientation','vertical','Location','eastoutside','Units','inches','Interpreter','latex')
leg.ItemTokenSize = [20,20];
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
return

figure('Position',[600,400,400,400])
    test_tip_pos=[];
    test_tip_pos=trainSet.tip_exp;
    plot3(test_tip_pos(:,2),test_tip_pos(:,3),test_tip_pos(:,4),'LineWidth',1,'LineStyle','-.','Color','k')
    hold on
    scatter3(test_tip_pos(1,2),test_tip_pos(1,3),test_tip_pos(1,4),'LineWidth',4,'Marker','hexagram','MarkerFaceColor','r')
    hold on
    plot3(xyz_estimation(:,1),xyz_estimation(:,2),xyz_estimation(:,3),'LineWidth',1,'LineStyle','-.','Color','b')
    hold on
    scatter3(xyz_estimation(1,1),xyz_estimation(1,2),xyz_estimation(1,3),'LineWidth',4,'Marker','hexagram','MarkerFaceColor','r')
    hold on
    xlim([-0.2,0.2])
    ylim([-0.2,0.2])
    zlim([-0.2,0.2])
    grid on
    title('Cam Frame')
    xlabel('X(m)')
    ylabel('Y(m)')
    zlabel('Z(m)')
    view(0,90)
    legend('Tip Exp','Start','Tip Est','Start')
theta_estimation=2*asin(sqrt(xyz_estimation(:,2).^2)./sqrt(xyz_estimation(:,2).^2+xyz_estimation(:,3).^2));
% figure('Position',[600,400,600,400])
%  plot(theta_estimation,'LineWidth',1,'LineStyle','-.','Color','k')
%  hold on
%  plot(abs(trainSet.theta_rad),'LineWidth',1,'LineStyle','--','Color','b')
end