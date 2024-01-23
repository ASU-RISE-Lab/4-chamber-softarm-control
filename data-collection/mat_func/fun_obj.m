function [cost_func_val] = func_obj(x0,u,x_ref,net,N)
        x_k = x0;
        cost_func_val = (x_ref(:,1)-x_k).^2;
        for jj = 2:N
        x_ref_next = x_ref(jj,:)';% 4x1 
        x_k_next = sim(net,[xk;u]);% 4x1
        x_k = x_k_next(1:4);%4x1
        e_k_next= x_ref_next - x_k_next;
        cost_func_val = cost_func_val + e_k_next.^2;
        end




end