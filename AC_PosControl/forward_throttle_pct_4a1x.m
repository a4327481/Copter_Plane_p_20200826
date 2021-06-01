function vel_forward_pct_out = forward_throttle_pct_4a1x()
%       in non-VTOL modes or modes without a velocity controller. We
%       don't use it in QHOVER or QSTABILIZE as they are the primary
%       recovery modes for a quadplane and need to be as simple as
%       possible. They will drift with the wind
%     work out the desired speed in forward direction
global Copter_Plane
global dt
global AC_PosControl
vel_forward_gain                   = Copter_Plane.vel_forward_gain;
vel_forward_tail_tilt_max          = Copter_Plane.vel_forward_tail_tilt_max;
vel_forward_integrator             = Copter_Plane.vel_forward_integrator;
pitch_target                       = AC_PosControl.pitch_target;

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
    
AC_PosControl.pitch_target                      = pitch_target;    
Copter_Plane.vel_forward_gain                   = vel_forward_gain;
Copter_Plane.vel_forward_min_pitch              = vel_forward_min_pitch;
Copter_Plane.vel_forward_tail_tilt_max          = vel_forward_tail_tilt_max;
Copter_Plane.vel_forward_integrator             = vel_forward_integrator;  
end

