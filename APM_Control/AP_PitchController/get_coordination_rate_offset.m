function [rate_offset, inverted]=get_coordination_rate_offset(  )  

%   get the rate offset in degrees/second needed for pitch in body frame
%   to maintain height in a coordinated turn.
%   
%   Also returns the inverted flag and the estimated airspeed in m/s for
%   use by the rest of the pitch controller
global roll
global pitch
global EAS2TAS
global GRAVITY_MSS
global aspeed
global HD;
global AP_rate_pitch
global Plane
global Copter_Plane

disable_AP_rate_pitch_roll_ff         = Copter_Plane.disable_AP_rate_pitch_roll_ff;
airspeed_min                          = Plane.airspeed_min;
roll_ff                               = AP_rate_pitch.roll_ff; 

if(disable_AP_rate_pitch_roll_ff)
    roll_ff            = 0; 
end


 	  bank_angle =  roll*HD;
	% limit bank angle between +- 80 deg if right way up
	
    if (abs(bank_angle) < radians(90))	 
	    bank_angle = constrain_value (bank_angle,-radians(80),radians(80));
        inverted = 0;  
    else  
		inverted = 1;	
        if (bank_angle > 0.0)  
			bank_angle = constrain_value(bank_angle,radians(100),radians(180));		  
        else  
			bank_angle = constrain_value(bank_angle,-radians(180),-radians(100));
        end
    end
	 
    if (abs(pitch*HD*100) > 7000)  
        % don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;    
    else  
        rate_offset = cos(pitch)*abs(HD*((GRAVITY_MSS / max((aspeed *EAS2TAS) ,  (airspeed_min*EAS2TAS))) * tan(bank_angle) * sin(bank_angle))) * roll_ff;
    end
    if (inverted)
		rate_offset = -rate_offset;
    end
    
  Copter_Plane.disable_AP_rate_pitch_roll_ff  = disable_AP_rate_pitch_roll_ff;
  Plane.airspeed_min                          = airspeed_min;
end

