function [x, dx, ddx] = Trapezoidal(pos_i, pos_f, dx_c, tf, t)

n = size(pos_i,1);
x   = zeros(n,1);
dx  = zeros(n,1);
ddx = zeros(n,1);


% repeat for each of the joint
for i=1:n
    delta = pos_f(i) - pos_i(i);
    dx_c(i) = sign(delta)*abs(dx_c(i));
    if (delta==0)
        x(i)   = pos_i(i);
        dx(i)  = 0;
        ddx(i) = 0;
    else
        % constraint verification for joint i
        dq_r = abs(delta/tf);
        err = (abs(dx_c(i)) <= dq_r)|(abs(dx_c(i)) > 2*dq_r);
        if (err==1)
            dx_c(i) = (2*dq_r*0.8);
        end
        
        % evaluates t_c
        t_c = tf - delta/dx_c(i);
        % evaluates ddq_c
        ddq_c = dx_c(i)/t_c;
        % if on the time slots
        if (t<=t_c)
            x(i)   = pos_i(i) + 0.5*ddq_c*t^2;
            dx(i)  = ddq_c*t;
            ddx(i) = ddq_c;
        elseif (t<=(tf-t_c))
            x(i)   = pos_i(i) + dx_c(i)*(t-0.5*t_c);
            dx(i)  = dx_c(i);
            ddx(i) = 0;
        elseif (t<=tf)
            x(i)   = pos_f(i) - 0.5*ddq_c*(t-tf)^2 ;
            dx(i)  = ddq_c*(tf-t);
            ddx(i) = -ddq_c;
        else
            x(i)   = pos_f(i);
            dx(i)  = 0;
            ddx(i) = 0;
        end
    end
end

end
