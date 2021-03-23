function check_for_failed_motor( throttle_thrust_best_plus_adj)
% check for failed motor
%   should be run immediately after output_armed_stabilizing
%   first argument is the sum of:
%      a) throttle_thrust_best_rpy : throttle level (from 0 to 1) providing maximum roll, pitch and yaw range without climbing
%      b) thr_adj: the difference between the pilot's desired throttle and throttle_thrust_best_rpy
%   records filtered motor output values in _thrust_rpyt_out_filt array
%   sets thrust_balanced to true if motors are balanced, false if a motor failure is detected
%   sets _motor_lost_index to index of failed motor

global dt
global AP_Motors

thrust_rpyt_out_filt              = AP_Motors.thrust_rpyt_out_filt;
motor_lost_index                  = AP_Motors.motor_lost_index;
thrust_boost                      = AP_Motors.thrust_boost;
thrust_balanced                   = AP_Motors.thrust_balanced;


 
    % record filtered and scaled thrust output for motor loss monitoring purposes
     alpha = 1.0 / (1.0 + 1/dt * 0.5);
    for i=1:4  
            thrust_rpyt_out_filt(i)=thrust_rpyt_out_filt(i) + alpha * (thrust_rpyt_out(i) - thrust_rpyt_out_filt(i));        
    end

      rpyt_high = 0.0;
      rpyt_sum  = 0.0;
      number_motors = 0.0;
    for i=1:4  
            number_motors =number_motors+ 1;
            rpyt_sum =rpyt_sum+ thrust_rpyt_out_filt(i);
            % record highest filtered thrust command
            if (thrust_rpyt_out_filt(i) > rpyt_high)  
                rpyt_high = thrust_rpyt_out_filt(i);
                % hold motor lost index constant while thrust boost is active
                if (~thrust_boost)  
                    motor_lost_index = i;              
                end
            end    
    end
         
     thrust_balance = 1.0;
    if (rpyt_sum > 0.1)  
        thrust_balance = rpyt_high * number_motors / rpyt_sum;
    end
    % ensure thrust balance does not activate for multirotors with less than 6 motors
    if (number_motors >= 6 && thrust_balance >= 1.5 && thrust_balanced)  
        thrust_balanced = false;
    end
    if (thrust_balance <= 1.25 && ~thrust_balanced)  
        thrust_balanced = true;
    end

    % check to see if thrust boost is using more throttle than _throttle_thrust_max
    if ((throttle_thrust_max * get_compensation_gain() > throttle_thrust_best_plus_adj) && (rpyt_high < 0.9) && thrust_balanced)  
        thrust_boost = false;
    end
	
	
AP_Motors.thrust_rpyt_out_filt              = thrust_rpyt_out_filt;
AP_Motors.motor_lost_index                  = motor_lost_index;
AP_Motors.thrust_boost                      = thrust_boost;
AP_Motors.thrust_balanced                   = thrust_balanced;
     
end

