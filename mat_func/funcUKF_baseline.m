function [xk_est,Pk] =funcUKF_baseline(xk_old,uk,yk,Pk_old,Q,R,lamda,dT)

% theta1 = xk_old(1);lc1 = xk_old(2);
% theta2 = xk_old(3);lc2 = xk_old(4);
% 
% pm11 = xk_old(5); pm12 = xk_old(6);
% pm21 = xk_old(7); pm22 = xk_old(8);

pd11 = uk(1);pd12 = uk(2);
pd21 = uk(3);pd22 = uk(4);

alphap = -0.9665; betap = 0.9698;
k1a2=-1.767; k1a1=17.55; k1a0 =33.471;
k2a2=10.25; k2a1=-325.1; k2a0 = 3299;
k3a2=-1.013; k3a1=11.55; k3a0 =4.419;
k4a2=15.34; k4a1=-474.1; k4a0 =4475;

d1a2=0.9725; d1a1=-11.2; d1a0 =38.471;
d2a2=-0.9725; d2a1=30.23; d2a0 =435.471;
d3a2=0.1125; d3a1=-1.2; d3a0=14.471;
d4a2=4.34; d4a1=-155.21; d4a0=2146;

% F_old = [theta1 - dT*(pm11 - pm12 + (theta1*(k1a0 + k1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + k1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)));
% lc1 + dT*(pm11 + pm12 - (lc1*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0));
% theta2 - dT*(pm21 - pm22 + (theta2*(k3a0 + k3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + k3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)));
% lc2 + dT*(pm21 + pm22 - (lc2*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0));
% pm11 + dT*(alphap*pm11 + betap*pd11);
% pm12 + dT*(alphap*pm12 + betap*pd12);
% pm21 + dT*(alphap*pm21 + betap*pd21);
% pm22 + dT*(alphap*pm22 + betap*pd22);];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = length(xk_old);
w = zeros(1,2*n+1);
for i = 1: 2*n+1
    w(i) = 1/(2*(n+lamda));
end
w(1) = lamda/(n+lamda);
xsigma = zeros(n,2*n+1);
L = (chol(Pk_old))';
xsigma(:,1) = xk_old;
for j=1:n
    xsigma(:,j+1) = xsigma(:,1) + sqrt(n+lamda)*L(:,j);
    xsigma(:,j+1+n) = xsigma(:,1) - sqrt(n+lamda)*L(:,j);
end
xsigmaminus = zeros(n,2*n+1);
for j =1:2*n+1
    theta1 = xsigma(1,j);lc1 = xsigma(2,j);
    theta2 = xsigma(3,j);lc2 = xsigma(4,j);

pm11 = xsigma(5,j); pm12 = xsigma(6,j);
pm21 = xsigma(7,j); pm22 = xsigma(8,j);

xsigmaminus(:,j) = [theta1 - dT*(pm11 - pm12 + (theta1*(k1a0 + k1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + k1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)));
lc1 + dT*(pm11 + pm12 - (lc1*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0));
theta2 - dT*(pm21 - pm22 + (theta2*(k3a0 + k3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + k3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)));
lc2 + dT*(pm21 + pm22 - (lc2*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0));
pm11 + dT*(alphap*pm11 + betap*pd11);
pm12 + dT*(alphap*pm12 + betap*pd12);
pm21 + dT*(alphap*pm21 + betap*pd21);
pm22 + dT*(alphap*pm22 + betap*pd22);];
end
xhatminus = zeros(n,1);
Pminus = zeros(n,n);
for j =1:2*n+1
    xhatminus = xhatminus+w(j)*xsigmaminus(:,j);
end
for j =1:2*n+1
    Pminus = Pminus+w(j)*(xsigmaminus(:,j)-xhatminus)*(xsigmaminus(:,j)-xhatminus)';
end
Pminus = Pminus+Q;
xsigma = zeros(n,2*n+1);
xsigma(:,1)=xhatminus;
L1 = (chol(Pminus))';
for j=1:n
    xsigma(:,j+1) = xsigma(:,1) + sqrt(n+lamda)*L1(:,j);
    xsigma(:,j+1+n) = xsigma(:,1) - sqrt(n+lamda)*L1(:,j);
end
yhat = zeros(n,1);
y = zeros(n,1);
for j =1:2*n+1
    y(:,j) = xsigma(:,j);
    yhat = yhat+w(j)*y(:,j);
end
Py = zeros(n,n);
Pxy = zeros(n,n);
for j =1:2*n+1
    Pxy = Pxy + w(j)*(xsigma(:,j)-xhatminus)*(y(:,j)-yhat)';
    Py  = Py + w(j)*(y(:,j)-yhat)*(y(:,j)-yhat)';
end
Py = Py+R;
Lk = Pxy*inv(Py);
xk_est= xhatminus + Lk*(yk -yhat);
Pk = Pminus - Lk*Py*Lk';
if isreal(xk_est) == 0
    xk_old
end
end