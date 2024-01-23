function [cost_func_val] = func_obj(x0,ns,ts,us,net,x_ref)
        f = func_update_cost(x0,ns,ts,us,net,x_ref);
        cost_func_val = f(5);
       
end