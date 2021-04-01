function servo_out=get_servo_out_yaw(  scaler,   disable_integrator)
 
global dt 
global HD
global gyro_z
global accel_y
global GRAVITY_MSS
global aspeed
global EAS2TAS
global roll
global AP_rate_yaw
global Plane
global Copter_Plane

airspeed_min      = Plane.airspeed_min;  
K_A               = AP_rate_yaw.K_A;
K_I               = AP_rate_yaw.K_I;
K_D               = AP_rate_yaw.K_D;
K_FF              = AP_rate_yaw.K_FF;
imax              = AP_rate_yaw.imax;

K_D_last          = AP_rate_yaw.K_D_last;
pid_info_I        = AP_rate_yaw.pid_info_I;
pid_info_D        = AP_rate_yaw.pid_info_D;
last_rate_hp_out  = AP_rate_yaw.last_rate_hp_out;
last_rate_hp_in   = AP_rate_yaw.last_rate_hp_in;
integrator        = AP_rate_yaw.integrator; 
last_out          = AP_rate_yaw.last_out;
disable_AP_rate_yaw_K_FF              = Copter_Plane.disable_AP_rate_yaw_K_FF;

if(disable_AP_rate_yaw_K_FF)
    K_FF = 0;
end

    servo_out=0;
    aspd_min=airspeed_min;
     if (aspd_min < 1)  
        aspd_min = 1;
     end
	
	  delta_time = dt;
	
	% Calculate yaw rate required to keep up with a constant height coordinated turn
 
	  bank_angle =roll;
	% limit bank angle between +- 80 deg if right way up
    if (abs(bank_angle) < 1.5707964)	 
	    bank_angle = constrain_value (bank_angle,-1.3962634,1.3962634);
    end

	 
    rate_offset = (GRAVITY_MSS / max(aspeed*EAS2TAS ,  (aspd_min*EAS2TAS))) * sin(bank_angle) * K_FF;

    % Get body rate vector (radians/sec)
	 
	
	% Get the accln vector (m/s^2)
 
	% Subtract the steady turn component of rate from the measured rate
	% to calculate the rate relative to the turn requirement in degrees/sec
	  rate_hp_in = HD*(gyro_z - rate_offset);
	
	% Apply a high-pass filter to the rate to washout any steady state error
	% due to bias errors in rate_offset
	% Use a cut-off frequency of omega = 0.2 rad/sec
	% Could make this adjustable by replacing 0.9960080 with (1 - omega * dt)
	  rate_hp_out = (1-0.2*dt)  *  last_rate_hp_out + rate_hp_in -  last_rate_hp_in;
	 last_rate_hp_out = rate_hp_out;
	 last_rate_hp_in = rate_hp_in;
	%Calculate input to integrator
	  integ_in = - K_I * (K_A * accel_y + rate_hp_out);
	
	% Apply integrator, but clamp input to prevent control saturation and freeze integrator below min FBW speed
	% Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	% Don't integrate if _K_D is zero as integrator will keep winding up	
    if (~disable_integrator && K_D > 0)  
		%only integrate if airspeed above min value	
        if (aspeed >  (aspd_min))		 
			% prevent the integrator from increasing if surface defln demand is above the upper limit	
            if ( last_out < -45)  
                 integrator =integrator+ max(integ_in * delta_time , 0);
            elseif (last_out > 45)  
                % prevent the integrator from decreasing if surface defln demand  is below the lower limit
                integrator =integrator+ min(integ_in * delta_time , 0);			  
            else  
               integrator=integrator + integ_in * delta_time;
            end
        end      
    else
		integrator = 0;    
    end
	 
    if (K_D < 0.0001)  
        % yaw damping is disabled, and the integrator is scaled by damping, so return 0
        servo_out=0;
        return ;
    end
     
	
    % Scale the integration limit
      intLimScaled = imax * 0.01 / (K_D * scaler * scaler);

    % Constrain the integrator state
    integrator = constrain_value (integrator, -intLimScaled, intLimScaled);
	
	% Protect against increases to _K_D during in-flight tuning from creating large control transients
	% due to stored integrator values
    if (K_D > K_D_last && K_D > 0)  
	    integrator = K_D_last/K_D * integrator;
    end
	K_D_last = K_D;
	
	% Calculate demanded rudder deflection, +Ve deflection yaws nose right
	% Save to last value before application of limiter so that integrator limiting
	% can detect exceedance next frame
	% Scale using inverse dynamic pressure (1/V^2)
	pid_info_I = K_D * integrator * scaler * scaler;
	pid_info_D = K_D * (-rate_hp_out) * scaler * scaler;
	last_out =  pid_info_I + pid_info_D;

	% Convert to centi-degrees and constrain
	 servo_out=constrain_value (last_out * 100, -4500, 4500);

     AP_rate_yaw.K_D_last                     = K_D_last;
     AP_rate_yaw.pid_info_I                   = pid_info_I;
     AP_rate_yaw.pid_info_D                   = pid_info_D;
     AP_rate_yaw.last_rate_hp_out             = last_rate_hp_out;
     AP_rate_yaw.last_rate_hp_in              = last_rate_hp_in;
     AP_rate_yaw.integrator                   = integrator;
     AP_rate_yaw.last_out                     = last_out;
     
end

