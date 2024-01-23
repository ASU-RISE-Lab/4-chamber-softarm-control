function [dpm,pm] = funcPmInfDef(t,pm,u6pm1,aup,bup,adow,bdow,varargin)
% alpha = -0.04; beta = 0.03768;
pd11 = u6pm1(1);
pd12 = u6pm1(2);
pd13 = u6pm1(3);
pd21 = u6pm1(4);
pd22 = u6pm1(5);
pd23 = u6pm1(6);
pm11 = pm(1); pm12 = pm(2); pm13 = pm(3);
pm21 = pm(4); pm22 = pm(5); pm23 = pm(6);
for i  = 1:6
    if u6pm1(i)-pm(i)>=0 % inflation
        dpm(i,1) = aup*pm(i) + bup*u6pm1(i);
    else
        dpm(i,1) = adow*pm(i)+ bdow*u6pm1(i);
    end

end