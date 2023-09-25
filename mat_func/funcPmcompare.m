function [dpm,pm] = funcPmcompare(t,pm,u6pm1,alpha,beta,varargin)
% alpha = -0.04; beta = 0.03768;
pd11 = u6pm1(1);
pd12 = u6pm1(2);
pd13 = u6pm1(3);
pd21 = u6pm1(4);
pd22 = u6pm1(5);
pd23 = u6pm1(6);
pm11 = pm(1); pm12 = pm(2); pm13 = pm(3);
pm21 = pm(4); pm22 = pm(5); pm23 = pm(6);
dpm = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';
end