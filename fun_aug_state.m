function dx = fun_aug_state(t,x,us,net,x_ref)
           % dt  = 1/60 from nerual net training sample.
        state_next = sim(net,[x(1:4);us]);%10x1
        dx = [(state_next(1:4) - x(1:4))*60 ;%10x1
                  (state_next(1:4) - x_ref')'*(state_next(1:4) - x_ref');
                 ];
end