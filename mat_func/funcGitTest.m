function par =funcGitTest(par)
n = 6; % DOF

% DH parameters symbols
q = sym('q', [n 1], 'real'); % generalized coordinates (joint angles)
d = sym('d', [n 1], 'real'); % link offsets
syms a1
syms g

Ti = cell(n,1);

DH = [0 -pi/2 d(1)  0;
      0  pi/2   0   q(2); 
      0     0 d(3)  0;
      0     0 a1  0;
      0 -pi/2 d(4)  0;
      0  pi/2   0   q(5); 
      0     0 d(6)  0;];
T = eye(4);
for i = 1:n
    temp = compute_dh_matrix(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
    T = T*temp;
    Ti{i} = T;
end
par.Ti = Ti;

Jw = arrayfun(@(x) sym(['Jw' num2str(x)], [3,n], 'real'), 1:n, 'UniformOutput', 0)';
Jw{1} = [[0;0;1] repmat([0;0;0],1,n-1)];
for i=2:n
       jw = [[0;0;1]];
       for j=1:i-1
           jw = [jw Ti{j}(1:3,3)];
       end
       jw = [jw repmat([0;0;0],1,n-i)];
       Jw{i} =jw;
end
c = arrayfun(@(x) [sym(['c' num2str(x) 'x'], 'real'), sym(['c' num2str(x) 'y'], 'real'), ...
    sym(['c' num2str(x) 'z'], 'real')]', 1:n, 'UniformOutput', 0)'

Jv = cell(n,1);
P = eye(4);
for i=1:n    
P = Ti{i}*[[1;0;0;0] [0;1;0;0] [0;0;1;0] [c{i};1]];
    x = P(1,4);
    y = P(2,4);
    z = P(3,4);
    for j=1:n
        Jv{i} = [Jv{i} [diff(x,q(j));diff(y,q(j));diff(z,q(j))]];
    end        
end
m = sym('m', [n 1], 'real');
par.