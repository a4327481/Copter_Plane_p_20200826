function stabilize_pitch(  speed_scaler)
%   this is the main pitch stabilization function. It takes the
%   previously set nav_pitch and calculates servo_out values to try to
%   stabilize the plane at the given attitude.
global pitch
global HD
global Test_w
global SRV_Channel
global Plane
global Copter_Plane
nav_pitch_cd             = Plane.nav_pitch_cd;
kff_throttle_to_pitch    = Plane.kff_throttle_to_pitch;
k_elevator               = SRV_Channel.k_elevator;
k_throttle               = SRV_Channel.k_throttle;
disable_integrator_pitch = Copter_Plane.disable_integrator_pitch;

% global curr_vel
%     int8_t force_elevator = takeoff_tail_hold();
%     if (force_elevator != 0)  
%         % we are holding the tail down during takeoff. Just convert
%         % from a percentage to a -4500..4500 centidegree angle
%         SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 45*force_elevator);
%         return;
%     end

  demanded_pitch = nav_pitch_cd  + k_throttle * kff_throttle_to_pitch;
  k_elevator=get_servo_out_pitch(demanded_pitch - pitch*HD*100,  speed_scaler, disable_integrator_pitch)+ Test_w.k_elevator;
  
%   set_output_scaled( k_elevator, get_servo_out_pitch(demanded_pitch - ahrs.pitch_sensor, 
%                                                                                            speed_scaler, 
%                                                                                            disable_integrator));

Plane.nav_pitch_cd                           = nav_pitch_cd;
Plane.kff_throttle_to_pitch                  = kff_throttle_to_pitch;
SRV_Channel.k_elevator                       = k_elevator;
SRV_Channel.k_throttle                       = k_throttle;
Copter_Plane.disable_integrator_pitch        = disable_integrator_pitch;

end

