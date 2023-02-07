function testData=funcGreyBoxSysID2seg(testData,par_set)
nlgr =funcBuildGreyBox2seg();   % get grey box model
%%%% Estimation Options
opt = nlgreyestOptions;
opt.Display='on';
opt.SearchOptions.MaxIterations = 2;
opt.SearchMethod='lsqnonlin';
%%%% calculate input output data
s1.l_t0 = 205.7796;s1.r_t0 = 209.0839; 
s2.l_t0 = 223.6281;s2.r_t0 = 222.3360;
r0 = 62.2254*0.75
green_t0 = 31.92;yellow_t0 = 46.33; purple_t0 =21.99;
s1.red_l = s1.l_t0 - green_t0;
s1.red_r = s1.r_t0 - green_t0;
s2.red_l = s2.l_t0 - yellow_t0 - purple_t0 - green_t0;
s2.red_r = s2.r_t0 - yellow_t0 - purple_t0 - green_t0;

s1.l_t = testData.enco_volts(:,1)/5*1000 - s1.red_l;
s1.r_t = testData.enco_volts(:,2)/5*1000 - s1.red_r;

s2.l_t = testData.enco_volts(:,4)/5*1000 - purple_t0 - s1.l_t - s2.red_l;
s2.r_t = testData.enco_volts(:,3)/5*1000 - purple_t0 - s1.r_t - s2.red_r;

s1.theta_wire_rad = (s1.r_t - s1.l_t)/r0;
s1.l_wire_mm = (s1.r_t + s1.l_t)/2;

s2.theta_wire_rad = (s2.r_t - s2.l_t)/r0;
s2.l_wire_mm = (s2.r_t + s2.l_t)/2;

tauy1 = testData.pd_psi(:,1) - testData.pd_psi(:,2);
fz1 = testData.pd_psi(:,1) + testData.pd_psi(:,2) + testData.pd_psi(:,3);
tauy2 = testData.pd_psi(:,4) - testData.pd_psi(:,5);
fz2 = testData.pd_psi(:,4) + testData.pd_psi(:,5) + testData.pd_psi(:,6);
%%%% Input Output Data 1
nlgr1=nlgr;
fprintf('Estimating\n')

z=iddata([s1.theta_wire_rad,s1.l_wire_mm/1000,s2.theta_wire_rad,s2.l_wire_mm/1000],...
    [tauy1,fz1,tauy2,fz2],par_set.Ts);

z.InputName=nlgr1.InputName;
z.InputUnit=nlgr1.InputUnit;
z.OutputName=nlgr1.OutputName;
z.OutputName=nlgr1.OutputName;
nlgr1.initial(1).Value=z.OutputData(1,1);
nlgr1.initial(3).Value=z.OutputData(1,2);
nlgr1.initial(5).Value=z.OutputData(1,3);
nlgr1.initial(7).Value=z.OutputData(1,4);
% figure
% plot(z);

nlgr1 = nlgreyest(z,nlgr1,opt);
fprintf('done')
figure
compare(z,nlgr1)
return
%%%%
% z2=iddata([testData.theta_rad(halfPt:end),testData.velocity_theta_rad(halfPt:end)],...
%     [testData.pm_MPa(halfPt:end,2:4),testData.beta(halfPt:end),testData.phi_rad(halfPt:end)],par_set.Ts);
% % z2=iddata([testData.theta_rad(1:end),testData.velocity_theta_rad(1:end)],...
% %     [testData.pm_MPa(1:end,2:4),testData.beta(1:end),testData.phi_rad(1:end)],par_set.Ts);
% z2.InputName=nlgr1.InputName;
% z2.InputUnit=nlgr1.InputUnit;
% z2.OutputName=nlgr1.OutputName;
% z2.OutputName=nlgr1.OutputName;
% nlgr2.initial(1).Value=z2.OutputData(1,1);
% nlgr2.initial(2).Value=z2.OutputData(1,2);
% nlgr2 = nlgreyest(z2,nlgr2,opt);
% figure
% compare(z2,nlgr2)
% 
% %     figure
% %     h_gcf = gcf;
% %     set(h_gcf,'DefaultLegendLocation','southeast');
% % for i = 1:z.Nu
% %    subplot(z.Nu, 1, i);
% %    plot(z.SamplingInstants, z.InputData(:,i));
% %    title(['Input #' num2str(i) ': ' z.InputName{i}]);
% %    xlabel('');
% %    axis tight;
% % end
% %     xlabel([z.Domain ' (' z.TimeUnit ')']);
% %
% %     figure
% %     h_gcf = gcf;
% %     set(h_gcf,'DefaultLegendLocation','southeast');
% %     h_gcf.Position = [100 100 795 634];
% % for i = 1:z.Ny
% %    subplot(z.Ny, 1, i);
% %    plot(z.SamplingInstants, z.OutputData(:,i));
% %    title(['Output #' num2str(i) ': ' z.OutputName{i}]);
% %    xlabel('');
% %    axis tight;
% % end
% % xlabel([z.Domain ' (' z.TimeUnit ')']);
% %%%% Estimate
% % figure
% 
% % %%
% 
% % figure
% % pe(z, nlgr1);
% testData.pi_grey=[nlgr1.Parameters(1).Value,nlgr1.Parameters(2).Value,nlgr1.Parameters(3).Value];
% testData.pi_grey2=[nlgr2.Parameters(1).Value,nlgr2.Parameters(2).Value,nlgr2.Parameters(3).Value];
% fprintf('Estimated [alpha,k,d] is [%.4f,%.4f,%.4f] \n',testData.pi_grey(1),testData.pi_grey(2),testData.pi_grey(3))
% fprintf('Estimated [alpha,k,d] is [%.4f,%.4f,%.4f] \n',testData.pi_grey2(1),testData.pi_grey2(2),testData.pi_grey2(3))
end