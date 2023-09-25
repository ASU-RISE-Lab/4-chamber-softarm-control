function dpm = funcPmOde45(t,pm,u6x1)
% alpha = -0.04; beta = 0.03768;
alpha = -0.9665; beta = 0.9698;
pd11 = u6x1(1);
pd12 = u6x1(2);
pd13 = u6x1(3);
pd21 = u6x1(4);
pd22 = u6x1(5);
pd23 = u6x1(6);
pm11 = pm(1); pm12 = pm(2); pm13 = pm(3);
pm21 = pm(4); pm22 = pm(5); pm23 = pm(6);
dpm = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';
end