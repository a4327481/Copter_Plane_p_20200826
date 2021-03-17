function  update_throttle_filter()
global armed
global throttle_in
global throttle_cutoff_frequency
global throttle_filter
global dt
global thrust_slew_time
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
end

