function  copter_run()
%copter run


global AC_PosControl
global SRV_Channel
global Plane
global Copter_Plane
global SINS
global plane_mode

aspeed                                = SINS.aspeed;
vel_desired                           = AC_PosControl.vel_desired;
target_yaw_rate                       = AC_PosControl.target_yaw_rate;

tail_tilt                             = SRV_Channel.tail_tilt;
k_aileron                             = SRV_Channel.k_aileron;
k_elevator                            = SRV_Channel.k_elevator;
k_rudder                              = SRV_Channel.k_rudder;
k_throttle                            = SRV_Channel.k_throttle;
p_plane_cp                            = Copter_Plane.p_plane_cp;
aspeed_cp                             = Copter_Plane.aspeed_cp;
disable_forward_throttle              = Copter_Plane.disable_forward_throttle;

if( (abs(vel_desired(1))>0) || (abs(vel_desired(2))>0)||(abs(target_yaw_rate)>0))
    temp_yaw_rate=0;
else
    temp_yaw_rate=get_weathervane_yaw_rate_cds();
end
switch plane_mode
    case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1,ENUM_plane_mode.V10s}
        if(disable_forward_throttle)
            vel_forward_pct_out = forward_throttle_pct_4a1x();%%tail xiu zheng
        else
            vel_forward_pct_out = forward_throttle_pct_4a1();%%tail xiu zheng
        end
        
        k_throttle=vel_forward_pct_out;
    case ENUM_plane_mode.V1000
        vel_forward_pct_out = forward_throttle_pctg();%%tail xiu zheng  
        tail_tilt=vel_forward_pct_out;   
end
update_vel_controller_xy();
update_z_controller();
roll_target                           = AC_PosControl.roll_target;
pitch_target                          = AC_PosControl.pitch_target;
target_yaw_rate                       = AC_PosControl.target_yaw_rate;
input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate+temp_yaw_rate);
rate_controller_run();
if(aspeed>aspeed_cp)
    nav_pitch_cd=pitch_target;
    nav_roll_cd=roll_target;
    Plane.nav_pitch_cd                                = nav_pitch_cd;
    Plane.nav_roll_cd                                 = nav_roll_cd;
    stabilize()
    k_aileron=k_aileron*p_plane_cp;
    k_elevator=k_elevator*p_plane_cp;
    k_rudder=k_rudder*p_plane_cp;
else
    k_aileron=0;
    k_elevator=0;
    k_rudder=0;
end

AP_Motors_output();

AC_PosControl.vel_desired                         = vel_desired;
AC_PosControl.roll_target                         = roll_target;
AC_PosControl.pitch_target                        = pitch_target;
AC_PosControl.target_yaw_rate                     = target_yaw_rate;

SRV_Channel.tail_tilt                             = tail_tilt;
SRV_Channel.k_aileron                             = k_aileron;
SRV_Channel.k_elevator                            = k_elevator;
SRV_Channel.k_rudder                              = k_rudder;
SRV_Channel.k_throttle                            = k_throttle;

Copter_Plane.p_plane_cp                           = p_plane_cp;
Copter_Plane.aspeed_cp                            = aspeed_cp;

end

