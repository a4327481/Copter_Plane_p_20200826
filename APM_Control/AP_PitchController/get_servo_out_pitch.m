function  servo_out=get_servo_out_pitch(  angle_err,   scaler,   disable_integrator)

global aspeed
global desired_rate_pitch
global AP_rate_pitch
gains_tau=AP_rate_pitch.gains_tau;
max_rate_neg=AP_rate_pitch.max_rate_neg;
gains_rmax=AP_rate_pitch.gains_rmax;
% Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
% A positive demand is up
% Inputs are: 
% 1) demanded pitch angle in centi-degrees
% 2) control gain scaler = scaling_speed / aspeed
% 3) boolean which is true when stabilise mode is active
% 4) minimum FBW airspeed (metres/sec)
% 5) maximum FBW airspeed (metres/sec)
%
 
	% Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
	% Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
	% Pitch rate offset is the component of turn rate about the pitch axis
 

    if (gains_tau < 0.1)  
        gains_tau=0.1;
    end
     

    [rate_offset,inverted] = get_coordination_rate_offset();
	
	% Calculate the desired pitch rate (deg/sec) from the angle error
	  desired_rate = angle_err * 0.01 / gains_tau;
	
	% limit the maximum pitch rate demand. Don't apply when inverted
	% as the rates will be tuned when upright, and it is common that
	% much higher rates are needed inverted	
    if (~inverted)  		
        if (max_rate_neg && desired_rate < -max_rate_neg)  
			desired_rate = -max_rate_neg;
        elseif (gains_rmax && desired_rate > gains_rmax)  
			desired_rate = gains_rmax;           
        end
    end
     
    if (inverted)
		desired_rate = -desired_rate;
    end

	% Apply the turn correction offset
	desired_rate = desired_rate + rate_offset;
    desired_rate_pitch=desired_rate;
    servo_out=get_rate_out_pitch(desired_rate, scaler, disable_integrator, aspeed);
 

end

