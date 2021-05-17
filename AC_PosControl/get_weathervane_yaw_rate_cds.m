function  weathervane_yaw_rate_cds=get_weathervane_yaw_rate_cds( )
global Copter_Plane
global AC_PosControl

weathervane_min_roll           = Copter_Plane.weathervane_min_roll;
weathervane_last_output        = Copter_Plane.weathervane_last_output;
weathervane_gain               = Copter_Plane.weathervane_gain;
yaw_rate_max                   = Copter_Plane.yaw_rate_max;
roll_target                    = AC_PosControl.roll_target;
 
%       we only do weathervaning in modes where we are doing VTOL
%       position control. We also don't do it if the pilot has given any
%       yaw input in the last 3 seconds.
    
 
     roll = roll_target / 100.0;
    if (abs(roll) < weathervane_min_roll)  
        Copter_Plane.weathervane_last_output = 0;
        weathervane_yaw_rate_cds=0;
        return ;
    end
     
    if (roll > 0)  
        roll=roll - weathervane_min_roll;
     else  
        roll=roll + weathervane_min_roll;
    end
    
      output = constrain_value((roll/45.0) * weathervane_gain, -1, 1); 
    weathervane_last_output = 0.98 * weathervane_last_output + 0.02 * output;

    % scale over half of yaw_rate_max. This gives the pilot twice the
    % authority of the weathervane controller
     weathervane_yaw_rate_cds=weathervane_last_output * (yaw_rate_max/2) * 100;
     
Copter_Plane.weathervane_min_roll           = weathervane_min_roll;
Copter_Plane.weathervane_last_output        = weathervane_last_output;
Copter_Plane.weathervane_gain               = weathervane_gain;
Copter_Plane.yaw_rate_max                   = yaw_rate_max;
AC_PosControl.roll_target                   = roll_target; 
end

