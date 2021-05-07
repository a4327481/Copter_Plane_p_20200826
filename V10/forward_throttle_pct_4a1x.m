function vel_forward_pct_out = forward_throttle_pct_4a1x()
%       in non-VTOL modes or modes without a velocity controller. We
%       don't use it in QHOVER or QSTABILIZE as they are the primary
%       recovery modes for a quadplane and need to be as simple as
%       possible. They will drift with the wind
%     work out the desired speed in forward direction
global pitch_target
global vel_forward_last_pct
global vel_forward_gain
 
global vel_forward_tail_tilt_max
global vel_forward_integrator
global dt

       pitch = pitch_target / 100.0;
       vel_forward_min_pitch=-12;
       fwd_vel_error=-(pitch - vel_forward_min_pitch)/15*0.3*0.5;
         
    if (pitch > 0)  
        fwd_vel_error = 0;
        vel_forward_integrator=vel_forward_integrator * 0.95;
    end
     
    vel_forward_integrator =vel_forward_integrator + fwd_vel_error * dt * vel_forward_gain ;
    vel_forward_integrator = constrain_value(vel_forward_integrator, 0, 0.6);
    vel_forward_pct_out=vel_forward_integrator;
end

