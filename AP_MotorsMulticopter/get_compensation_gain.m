function    ret=get_compensation_gain()
global AP_Motors

air_density_ratio  = AP_Motors.air_density_ratio;
lift_max           = AP_Motors.lift_max;
% avoid divide by zero
if(lift_max<=0)
    ret=1;
    return;
end

ret=1/lift_max;
% air density ratio is increasing in density / decreasing in altitude
if (air_density_ratio > 0.3 && air_density_ratio < 1.5)
    ret=ret *1.0 / constrain_value(air_density_ratio,0.5,1.25);
end
end

