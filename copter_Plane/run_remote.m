function  run_remote()
global algo_remote_ct_st
global AP_Motors
global AC_PosControl
global Plane
global AP_L1
global AP_TECS
global SRV_Channel
global Copter_Plane

roll_target                           = AC_PosControl.roll_target;
pitch_target                          = AC_PosControl.pitch_target;
target_yaw_rate                       = AC_PosControl.target_yaw_rate;
throttle_in                           = AP_Motors.throttle_in;
nav_pitch_cd                          = Plane.nav_pitch_cd;
latAccDem                             = AP_L1.latAccDem;
throttle_dem                          = AP_TECS.throttle_dem;
tail_tilt                             = SRV_Channel.tail_tilt;
pwm_tail                              = SRV_Channel.pwm_tail;

Mode                                  = Copter_Plane.Mode;
climb_rate_cms                        = Copter_Plane.climb_rate_cms;
roll_target_pilot                     = Copter_Plane.roll_target_pilot;
pitch_target_pilot                    = Copter_Plane.pitch_target_pilot;

dead=0.05;
if(algo_remote_ct_st.isRemoteConnected||algo_remote_ct_st.Mode==ENUM_Mode.AUTO_TEST||algo_remote_ct_st.Mode==ENUM_Mode.AUTO)
    Mode=algo_remote_ct_st.Mode;
else
    if(algo_remote_ct_st.Mode==ENUM_Mode.Copter_STABILIZE||algo_remote_ct_st.Mode==ENUM_Mode.Copter_ALT_HOLD||algo_remote_ct_st.Mode==ENUM_Mode.Copter_POS_HOLD||algo_remote_ct_st.Mode==ENUM_Mode.Copter_Plane_MANUAL)
        Mode=ENUM_Mode.Copter_POS_HOLD;
    elseif((algo_remote_ct_st.Mode==ENUM_Mode.MANUAL||algo_remote_ct_st.Mode==ENUM_Mode.Plane_STABILIZE||algo_remote_ct_st.Mode==ENUM_Mode.Plane_TECS||algo_remote_ct_st.Mode==ENUM_Mode.Plane_L1_WAYPOINT||algo_remote_ct_st.Mode==ENUM_Mode.Plane_L1_LOITER))
        Mode=ENUM_Mode.Plane_L1_LOITER;
    end
end
algo_remote_ct_st.roll    = deadzonef(algo_remote_ct_st.roll,dead,1);
algo_remote_ct_st.pitch   = deadzonef(algo_remote_ct_st.pitch,dead,1);
algo_remote_ct_st.yaw     = deadzonef(algo_remote_ct_st.yaw,dead,1);
switch Mode
    case ENUM_Mode.Copter_STABILIZE
        roll_target=algo_remote_ct_st.roll*4500;
        pitch_target=algo_remote_ct_st.pitch*4500;
        target_yaw_rate=algo_remote_ct_st.yaw*20000;
        throttle_in=algo_remote_ct_st.throttle;
    case ENUM_Mode.Copter_ALT_HOLD
        roll_target=algo_remote_ct_st.roll*4500;
        pitch_target=algo_remote_ct_st.pitch*4500;
        target_yaw_rate=algo_remote_ct_st.yaw*20000;
        climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;
    case ENUM_Mode.Copter_POS_HOLD
        roll_target_pilot=algo_remote_ct_st.roll*5000;
        pitch_target_pilot=-algo_remote_ct_st.pitch*5000;
        target_yaw_rate=algo_remote_ct_st.yaw*20000;
        climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;
    case ENUM_Mode.Plane_STABILIZE
        latAccDem=algo_remote_ct_st.roll*9.8;
        nav_pitch_cd=algo_remote_ct_st.pitch*2000;
        throttle_dem=algo_remote_ct_st.throttle;
    case ENUM_Mode.Plane_TECS
        latAccDem=algo_remote_ct_st.roll*9.8;
        climb_rate_cms=algo_remote_ct_st.pitch*600;
    case ENUM_Mode.Plane_L1_WAYPOINT
        climb_rate_cms=algo_remote_ct_st.pitch*600;
    case ENUM_Mode.Copter_Plane_MANUAL
        roll_target=algo_remote_ct_st.roll*4500;
        pitch_target=algo_remote_ct_st.pitch*4500;
        target_yaw_rate=algo_remote_ct_st.yaw*20000;
        climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;
        tail_tilt=algo_remote_ct_st.tail_anglein;
        pwm_tail=algo_remote_ct_st.tail_throttle_pwm;
    case ENUM_Mode.Plane_L1_LOITER
        climb_rate_cms=algo_remote_ct_st.pitch*600;
end

AC_PosControl.roll_target                           = roll_target;
AC_PosControl.pitch_target                          = pitch_target;
AC_PosControl.target_yaw_rate                       = target_yaw_rate;
AP_Motors.throttle_in                               = throttle_in;
Plane.nav_pitch_cd                                  = nav_pitch_cd;
AP_L1.latAccDem                                     = latAccDem;
AP_TECS.throttle_dem                                = throttle_dem;
SRV_Channel.tail_tilt                               = tail_tilt;
SRV_Channel.pwm_tail                                = pwm_tail;
Copter_Plane.Mode                                   = Mode;
Copter_Plane.climb_rate_cms                         = climb_rate_cms;
Copter_Plane.roll_target_pilot                      = roll_target_pilot;
Copter_Plane.pitch_target_pilot                     = pitch_target_pilot;
end

