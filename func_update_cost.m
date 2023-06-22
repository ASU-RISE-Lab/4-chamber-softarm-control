function f = func_update_cost(x0,ns,ts,us,net,x_ref)

% integration
z0 = [x0(1:4);(x_ref - x0(1:4)')*(x_ref - x0(1:4)')'];

%% applying control for each stage
% remember control is piecewise affine and in equal intervals.
% us 6xns
optODE = odeset('RelTol',1e-8,'AbsTol',1e-8);
uk =reshape(us,[6,ns]);

for ks = 1:ns
    [~,zs] = ode15s(@(t,x)fun_aug_state(t,x,uk(:,ks),net,x_ref),[ts(ks) ts(ks+1)], z0, optODE);
    z0 = zs(end,:)';% at ts(ks +1 )

end

% functions
f = zs(end,:)';

%%% to check this, run
% ns = 2;
% ts = [0 1 2]; % equally spaced time: 0s to 1s, 1s to 2s
% x0 = [1 1 0];
% ws = [10 -10];
% fun(x0,ns,ts,ws)