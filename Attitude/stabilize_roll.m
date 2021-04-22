function  stabilize_roll(  speed_scaler)
%   this is the main roll stabilization function. It takes the
%   previously set nav_roll calculates roll servo_out to try to
%   stabilize the plane at the given roll

 global HD
 global Test_w
 global Plane
 global SRV_Channel
 global Copter_Plane
 global SINS
 
 
 roll                            =SINS.roll;
 inverted_flight                 = Plane.inverted_flight;
 nav_roll_cd                     = Plane.nav_roll_cd;
 disable_AP_roll_integrator         = Copter_Plane.disable_AP_roll_integrator;
 
    if (inverted_flight)  
        % we want to fly upside down. We need to cope with wrap of
        % the roll_sensor interfering with wrap of nav_roll, which
        % would really confuse the PID code. The easiest way to
        % handle this is to ensure both go in the same direction from
        % zero
        nav_roll_cd=nav_roll_cd + 18000;
        if (roll < 0) 
            nav_roll_cd=nav_roll_cd - 36000;
        end
    end
    k_aileron=get_servo_out_roll(nav_roll_cd - roll*HD*100, speed_scaler, disable_AP_roll_integrator)+Test_w.k_aileron;
     
     
%     SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, 
%                                                                                          speed_scaler, 
%                                                                                          disable_integrator));
 Plane.inverted_flight                 = inverted_flight;
 Plane.nav_roll_cd                     = nav_roll_cd ;
 SRV_Channel.k_aileron                 = k_aileron;
 Copter_Plane.disable_AP_roll_integrator  = disable_AP_roll_integrator;
end

