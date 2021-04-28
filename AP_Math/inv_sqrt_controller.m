function  output = inv_sqrt_controller(  intput,   p,   D_max)

if ((D_max<=0) && is_zero(p))
    output = 0 ;
    return
end

if (D_max<=0)
    % second order limit is zero or negative.
    output = intput / p;
    return
end

if (is_zero(p))
    if(intput>0)
        % P term is zero but we have a second order limit.
        output= (intput)^2/(2*D_max);
        return
    else
        output= -(intput)^2/(2*D_max);
        return
    end
end
    
    % both the P and second order limit have been defined.
    linear_out = D_max / p;
    if (intput > linear_out)
        output=(intput)^2/(2*D_max) + D_max/(2*(p)^2);
        return
    elseif(intput <- linear_out)
        output=-(intput)^2/(2*D_max) - D_max/(2*(p)^2);
        return
    end
    
    output= intput / p;
    
    
end

