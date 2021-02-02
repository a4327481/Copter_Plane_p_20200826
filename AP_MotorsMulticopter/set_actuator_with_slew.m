function actuator_output_o=set_actuator_with_slew(   actuator_output,input)
global slew_up_time
global slew_dn_time
global dt
%  adds slew rate limiting to actuator output     
%     If MOT_SLEW_UP_TIME is 0 (default), no slew limit is applied to increasing output.
%     If MOT_SLEW_DN_TIME is 0 (default), no slew limit is applied to decreasing output.
%     MOT_SLEW_UP_TIME and MOT_SLEW_DN_TIME are constrained to 0.0~0.5 for sanity.
%     If spool mode is shutdown, no slew limit is applied to allow immediate disarming of motors.
     

%      Output limits with no slew time applied
     output_slew_limit_up = 1.0;
     output_slew_limit_dn = 0.0;

%      If MOT_SLEW_UP_TIME is set, calculate the highest allowed new output value, constrained 0.0~1.0
    if ((slew_up_time)>0)  
         output_delta_up_max = 1.0 / (constrain_value(slew_up_time, 0.0, 0.5) * (1/dt));
        output_slew_limit_up = constrain_value(actuator_output + output_delta_up_max, 0.0, 1.0);
    end

%      If MOT_SLEW_DN_TIME is set, calculate the lowest allowed new output value, constrained 0.0~1.0
    if ((slew_dn_time)>0)  
         output_delta_dn_max = 1.0 / (constrain_value(slew_dn_time, 0.0, 0.5) * (1/dt));
        output_slew_limit_dn = constrain_value(actuator_output - output_delta_dn_max, 0.0, 1.0);
    end

%      Constrain change in output to within the above limits
    actuator_output_o = constrain_value(input, output_slew_limit_dn, output_slew_limit_up);
 

end

