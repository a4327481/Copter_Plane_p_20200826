function vel_forward_pct_out = forward_throttle_pct()
%       in non-VTOL modes or modes without a velocity controller. We
%       don't use it in QHOVER or QSTABILIZE as they are the primary
%       recovery modes for a quadplane and need to be as simple as
%       possible. They will drift with the wind
%     work out the desired speed in forward direction
global pitch_target
global vel_forward_last_pct
global vel_forward_gain
global vel_forward_min_pitch
global vel_forward_tail_tilt_max

global  AC_PosControl
pitch_target                        = AC_PosControl.pitch_target;

     pitch = pitch_target / 100.0;
    if (pitch > vel_forward_min_pitch)  
        vel_forward_last_pct = 0;
        vel_forward_pct_out=0;
        return ;
    end
     
       pitch=pitch - vel_forward_min_pitch;
       
      output = constrain_value((pitch/25.0) * vel_forward_gain, -1, 0); 
    vel_forward_last_pct = 0.98 * vel_forward_last_pct + 0.02 * output;
    % scale over half of yaw_rate_max. This gives the pilot twice the
    % authority of the weathervane controller
     vel_forward_pct_out=vel_forward_last_pct * (vel_forward_tail_tilt_max) ;
     
     
          AC_PosControl.pitch_target            =pitch_target;    

end

