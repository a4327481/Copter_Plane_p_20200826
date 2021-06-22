function  update_throttle_filter()
global Copter_Plane
global dt
global AP_Motors

armed                                  = Copter_Plane.armed;
throttle_cutoff_frequency              = AP_Motors.throttle_cutoff_frequency;
thrust_slew_time                       = AP_Motors.thrust_slew_time;
throttle_in                            = AP_Motors.throttle_in;
throttle_filter                        = AP_Motors.throttle_filter;

if (armed)
    throttle_filter_temp=throttle_filter;
    throttle_filter= throttle_filter+(throttle_in - throttle_filter) * get_filt_alpha(throttle_cutoff_frequency);
    throttle_filter=constrain_value(throttle_filter,0,1);
    
    thrust_dt=dt/thrust_slew_time/3;
    throttle_filter=constrain_value(throttle_filter,throttle_filter_temp-thrust_dt,throttle_filter_temp+thrust_dt);
    if(throttle_filter>0.9)
        throttle_filter=0.9;
    end 
else
    throttle_filter=0;
end

AP_Motors.throttle_filter                        = throttle_filter;


end

