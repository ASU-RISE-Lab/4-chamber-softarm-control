function [] = funcPlotRawData(testData)
    close all;
    figure(1)
    title('xyz pos in cam frame')
    plot3(testData.rigid_1_pose(:,1),testData.rigid_1_pose(:,2),testData.rigid_1_pose(:,3),'r')
    hold on
    plot3(testData.rigid_2_pose(:,1),testData.rigid_2_pose(:,2),testData.rigid_2_pose(:,3),'b')
    hold on
    plot3(testData.rigid_3_pose(:,1),testData.rigid_3_pose(:,2),testData.rigid_3_pose(:,3),'k')
    hold on
    legend('R1','R2','R3')
    xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');

    figure(2)
    title('xz pos in cam frame')
    plot(testData.rigid_1_pose(:,1),testData.rigid_1_pose(:,3),'r')
    hold on
    plot(testData.rigid_2_pose(:,1),testData.rigid_2_pose(:,3),'b')
    hold on
    plot(testData.rigid_3_pose(:,1),testData.rigid_3_pose(:,3),'k')
    hold on
    legend('R1','R2','R3')
    xlabel('x (m)');ylabel('z (m)');

    figure(3)
    title('encoder readings')
    plot(testData.enco_volts(:,1))
    hold on
    plot(testData.enco_volts(:,2))
    hold on
    plot(testData.enco_volts(:,4))
    hold on
    plot(testData.enco_volts(:,3))
    hold on
    legend('s1-l','s1-r','s2-l','s2-r')
    xlabel('index (30Hz)');ylabel('Volts');
end