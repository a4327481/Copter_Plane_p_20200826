%mode_Multicopter
load IOBusInfo_byc_20200903.mat
load IOBusInfo_V1000.mat
mode_data();
sin_step_INIT();
plane_mode=ENUM_plane_mode.V10;
m_kg_V1000=5;
Jx=186222*1e-6;
Jy=164400*1e-6;
Jz=336920*1e-6;
J_V1000=diag([Jx Jy Jz]);

m_kg_V10=26;
Jx_v10=4.17029;
Jy_v10=8.07546;
Jz_v10=4.01077;
Jxz_V10=0.1849;
J_V10=diag([Jx_v10 Jy_v10 Jz_v10]);
switch plane_mode
    case ENUM_plane_mode.V1000
        m_kg=m_kg_V1000;
        J=J_V1000;
    case ENUM_plane_mode.V10
        m_kg=m_kg_V10;
        J=J_V10;
    case ENUM_plane_mode.V10s
        m_kg=m_kg_V10;
        J=J_V10;
    otherwise
        m_kg=m_kg_V10;
        J=J_V10;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rho                                                      = 1;
gyro                                                     = [00/HD -200/HD 0];
Euler                                                    = [5/HD 5/HD 30/HD];
velocity                                                 = [0 0 0];
position                                                 = [0 0 -100];
%
dt                                                       = 0.005;
HD                                                       = 180/pi;
GRAVITY_MSS                                              = 9.80665;
LOCATION_SCALING_FACTOR                                  = 0.011131884502145034;% 1e-7*地球半径/HD
LOCATION_SCALING_FACTOR_INV                              = 89.83204953368922;
plane_mode                                               = ENUM_plane_mode.V10;

SRV_Channel.k_rudder                                     = 0;
SRV_Channel.k_aileron                                    = 0;
SRV_Channel.k_elevator                                   = 0;
SRV_Channel.k_throttle                                   = 0;
SRV_Channel.k_flap                                       = 0;
SRV_Channel.tail_tilt                                    = 0;
SRV_Channel.pwm_tail                                     = 1000;
SRV_Channel.pwm_out                                      = [1000 1000 1000 1000]+0.0*1000;
AP_Motors.throttle_in                                    = 0.0                            ;
AP_Motors.throttle_filter                                = 0.0                            ;

Curr_sate.accel_x                                        = 0;
Curr_sate.accel_y                                        = 0;
Curr_sate.accel_z                                        = 0;
[SINS.gyro_x ,SINS.gyro_y ,SINS.gyro_z]                  = deal(gyro(1),gyro(2),gyro(3));
[Curr_sate.rolld ,Curr_sate.pitchd ,Curr_sate.yawd]      = deal(Euler(1)*HD ,Euler(2)*HD ,Euler(3)*HD);
Curr_sate.EAS_Algo                                       = 0;
Curr_sate.EAS2TAS_Algo                                   = 1;
Curr_sate.curLLA                                         = [0 0 position(3)];
Curr_sate.NAV_alt                                        = position(3);
Curr_sate.PathModeOut.headingCmd                         = 0;
Curr_sate.PathModeOut.groundspeedCmd                     = 0;
Curr_sate.PathModeOut.heightCmd                          = 0;
Curr_sate.PathModeOut.flightTaskMode                     = ENUM_FlightTaskMode.GroundStandByMode;
Curr_sate.PathModeOut.flightControlMode                  = ENUM_FlightControlMode.GroundStandByControlMode;
Curr_sate.PathModeOut.maxClimbSpeed                      = 0;
Curr_sate.PathModeOut.turnCenterLL                       = [40,100.2];
Curr_sate.PathModeOut.prePathPoint_LLA                   = [40, 100,   0];
Curr_sate.PathModeOut.curPathPoint_LLA                   = [40, 100.01,0];
Curr_sate.PathModeOut.rollCmd                            = 0;
loc.num=1:20;
loc.lat=[
    40*1e7;
    40.01*1e7;
    40.02*1e7;
    40.02*1e7;
    40.01*1e7;
    zeros(15,1)
    ];
loc.lon=[
    100*1e7;
    100.01*1e7;
    100.01*1e7;
    100.02*1e7;
    100.02*1e7;
    zeros(15,1)
    ];
Curr_sate.wp_in(:,1)   = loc.num;
Curr_sate.wp_in(:,2)   = loc.lat;
Curr_sate.wp_in(:,3)   = loc.lon;
Curr_sate.wp_in(:,4)   = zeros(20,1);

PathModeOut_sl.headingCmd                                = 10000;
PathModeOut_sl.groundspeedCmd                            = 0;
PathModeOut_sl.heightCmd                                 = 10000;
PathModeOut_sl.flightTaskMode                            = ENUM_FlightTaskMode.GroundStandByMode;
PathModeOut_sl.maxClimbSpeed                             = 300;
PathModeOut_sl.turnCenterLL                              = [40,100]*1e7;
PathModeOut_sl.prePathPoint_LLA                          = [0 0 0];
PathModeOut_sl.curPathPoint_LLA                          = [0 0 0];

algo_remote_ct_st.isRemoteConnected                      = 1;
algo_remote_ct_st.Mode                                   = ENUM_Mode.MANUAL;
algo_remote_ct_st.roll                                   = 0;
algo_remote_ct_st.pitch                                  = 0;
algo_remote_ct_st.yaw                                    = 0;
algo_remote_ct_st.throttle                               = 0;
algo_remote_ct_st.tail_anglein                           = 0;
algo_remote_ct_st.tail_throttle_pwm                      = 0;

algo_dbg_param.headingCmd                                = 0;
algo_dbg_param.groundspeedCmd                            = 0;
algo_dbg_param.heightCmd                                 = 0;
algo_dbg_param.flightTaskMode                            = ENUM_FlightTaskMode.Rotor2Fix_Mode;
algo_dbg_param.maxClimbSpeed                             = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%控制参数
AC_rate_roll_pid.kp                                      = 0.2    ;%ATC_RAT_RLL_P
AC_rate_roll_pid.ki                                      = 0.1    ;%ATC_RAT_RLL_I
AC_rate_roll_pid.kd                                      = 0      ;%ATC_RAT_RLL_D
AC_rate_roll_pid.kff                                     = 0      ;%ATC_RAT_RLL_FF
AC_rate_roll_pid.kimax                                   = 0.25   ;%ATC_RAT_RLL_IMAX
AC_rate_roll_pid.filt_T_hz                               = 20     ;%ATC_RAT_RLL_FLTT
AC_rate_roll_pid.filt_E_hz                               = 20     ;%ATC_RAT_RLL_FLTE
AC_rate_roll_pid.filt_D_hz                               = 20     ;%ATC_RAT_RLL_FLTD
AC_rate_roll_pid.slew_rate_max                           = 0      ;%ATC_RAT_RLL_SMAX
AC_rate_roll_pid.slew_rate_tau                           = 0.1    ;%ATC_RAT_RLL_STAU
AC_rate_pitch_pid.kp                                     = 0.28   ;%ATC_RAT_PIT_P
AC_rate_pitch_pid.ki                                     = 0.1    ;%ATC_RAT_PIT_I
AC_rate_pitch_pid.kd                                     = 0      ;%ATC_RAT_PIT_D
AC_rate_pitch_pid.kff                                    = 0      ;%ATC_RAT_PIT_FF
AC_rate_pitch_pid.kimax                                  = 0.25   ;%ATC_RAT_PIT_IMAX
AC_rate_pitch_pid.filt_T_hz                              = 20     ;%ATC_RAT_PIT_FLTT
AC_rate_pitch_pid.filt_E_hz                              = 20     ;%ATC_RAT_PIT_FLTE
AC_rate_pitch_pid.filt_D_hz                              = 20     ;%ATC_RAT_PIT_FLTD
AC_rate_pitch_pid.slew_rate_max                          = 0      ;%ATC_RAT_PIT_SMAX
AC_rate_pitch_pid.slew_rate_tau                          = 0.1    ;%ATC_RAT_PIT_STAU
AC_rate_yaw_pid.kp                                       = 0.19   ;%ATC_RAT_YAW_P
AC_rate_yaw_pid.ki                                       = 0.0    ;%ATC_RAT_YAW_I
AC_rate_yaw_pid.kd                                       = 0      ;%ATC_RAT_YAW_D
AC_rate_yaw_pid.kff                                      = 0      ;%ATC_RAT_YAW_FF
AC_rate_yaw_pid.kimax                                    = 0.0    ;%ATC_RAT_YAW_IMAX
AC_rate_yaw_pid.filt_T_hz                                = 20     ;%ATC_RAT_YAW_FLTT
AC_rate_yaw_pid.filt_E_hz                                = 20     ;%ATC_RAT_YAW_FLTE
AC_rate_yaw_pid.filt_D_hz                                = 20     ;%ATC_RAT_YAW_FLTD
AC_rate_yaw_pid.slew_rate_max                            = 0      ;%ATC_RAT_YAW_SMAX
AC_rate_yaw_pid.slew_rate_tau                            = 0.1    ;%ATC_RAT_YAW_STAU
AC_Attitude.p_angle_roll                                 = 3.3    ;%ATC_ANG_RLL_P
AC_Attitude.p_angle_pitch                                = 3      ;%ATC_ANG_PIT_P
AC_Attitude.p_angle_yaw                                  = 3      ;%ATC_ANG_YAW_P
AC_Attitude.input_tc                                     = 0.3    ;%ATC_ANG_LIM_TC
AC_PosControl.p_pos_z                                    = 1      ;%PSC_POSZ_P
AC_PosControl.p_vel_z                                    = 1      ;%PSC_VELZ_P
AC_PosControl.pid_accel_z.kp                             = 0.9    ;%PSC_ACCZ_P
AC_PosControl.pid_accel_z.ki                             = 0.18   ;%PSC_ACCZ_I
AC_PosControl.pid_accel_z.kd                             = 0.08   ;%PSC_ACCZ_D 
AC_PosControl.pid_accel_z.kff                            = 0      ;%PSC_ACCZ_FF
AC_PosControl.pid_accel_z.kimax                          = 200    ;%PSC_ACCZ_IMAX
AC_PosControl.pid_accel_z.filt_T_hz                      = 20     ;%PSC_ACCZ_FLTT
AC_PosControl.pid_accel_z.filt_E_hz                      = 4      ;%PSC_ACCZ_FLTE
AC_PosControl.pid_accel_z.filt_D_hz                      = 20     ;%PSC_ACCZ_FLTD
AC_PosControl.pid_accel_z.slew_rate_max                  = 0      ;%PSC_ACCZ_SMAX
AC_PosControl.pid_accel_z.slew_rate_tau                  = 0.1    ;%PSC_ACCZ_STAU
AC_PosControl.p_pos_xy                                   = 1.1    ;%PSC_POSXY_P
AC_PosControl.pid_vel_xy.kp                              = 1.1    ;%PSC_VELXY_P
AC_PosControl.pid_vel_xy.ki                              = 0.15   ;%PSC_VELXY_I
AC_PosControl.pid_vel_xy.kd                              = 0.12   ;%PSC_VELXY_D 
AC_PosControl.pid_vel_xy.kff                             = 0      ;%PSC_VELXY_FF
AC_PosControl.pid_vel_xy.kimax                           = 1000   ;%PSC_VELXY_IMAX
AC_PosControl.pid_vel_xy.filt_T_hz                       = 20     ;%PSC_VELXY_FLTT
AC_PosControl.pid_vel_xy.filt_E_hz                       = 2      ;%PSC_VELXY_FLTE
AC_PosControl.pid_vel_xy.filt_D_hz                       = 5      ;%PSC_VELXY_FLTD
%  AC_PosControl.pid_vel_xy.slew_rate_max                = 0;
%  AC_PosControl.pid_vel_xy.slew_rate_tau                = 0.1;
AP_rate_roll.gains_tau                                   = 0.5    ;%RLL2SRV_TCONST
AP_rate_roll.gains_rmax                                  = 0      ;%RLL2SRV_RMAX
AP_rate_roll.gains_P                                     = 1.27   ;%RLL2SRV_P
AP_rate_roll.gains_D                                     = 0.22   ;%RLL2SRV_D 
AP_rate_roll.gains_I                                     = 0      ;%RLL2SRV_I
AP_rate_roll.gains_FF                                    = 0      ;%RLL2SRV_FF    
AP_rate_roll.gains_imax                                  = 3000   ;%RLL2SRV_IMAX
AP_rate_roll.slew_rate_max                               = 0      ;%RLL2SRV_SMAX 
AP_rate_roll.slew_rate_tau                               = 0.1    ;%RLL2SRV_TAU
AP_rate_pitch.gains_tau                                  = 1      ;%PTCH2SRV_TCONST
AP_rate_pitch.gains_rmax                                 = 0      ;%PTCH2SRV_RMAX_UP
AP_rate_pitch.max_rate_neg                               = 0      ;%PTCH2SRV_RMAX_DN
AP_rate_pitch.gains_P                                    = 1.2    ;%PTCH2SRV_P 
AP_rate_pitch.gains_D                                    = 0.45   ;%PTCH2SRV_D
AP_rate_pitch.gains_I                                    = 0.20   ;%PTCH2SRV_I    
AP_rate_pitch.gains_FF                                   = 0      ;%PTCH2SRV_FF
AP_rate_pitch.gains_imax                                 = 3000   ;%PTCH2SRV_IMAX
AP_rate_pitch.roll_ff                                    = 0.85   ;%PTCH2SRV_RLL
AP_rate_pitch.slew_rate_max                              = 0      ;%PTCH2SRV_SMAX 
AP_rate_pitch.slew_rate_tau                              = 0.1    ;%PTCH2SRV_TAU 
AP_rate_yaw.K_A                                          = 0.25   ;%YAW2SRV_SLIP
AP_rate_yaw.K_I                                          = 0.15   ;%YAW2SRV_INT   
AP_rate_yaw.K_D                                          = 0.35   ;%YAW2SRV_DAMP
AP_rate_yaw.K_FF                                         = 1      ;%YAW2SRV_RLL
AP_rate_yaw.imax                                         = 1500   ;%YAW2SRV_IMAX                                                                
AP_TECS.maxClimbRate                                     = 5      ;%TECS_CLMB_MAX
AP_TECS.minSinkRate                                      = 3      ;%TECS_SINK_MIN 
AP_TECS.timeConstant                                     = 3      ;%TECS_TIME_CONST
AP_TECS.integGain                                        = 0.25   ;%TECS_INTEG_GAIN
AP_TECS.rollComp                                         = 20     ;%TECS_RLL2THR
AP_TECS.ptchDamp                                         = 0.7    ;%TECS_PTCH_DAMP
AP_TECS.thrDamp                                          = 0.7    ;%TECS_THR_DAMP 
AP_TECS.throttle_cruise                                  = 40     ;%TRIM_THROTTLE
AP_TECS.throttle_slewrate                                = 100    ;%THR_SLEWRATE
AP_TECS.maxSinkRate                                      = 7      ;%TECS_SINK_MAX
AP_TECS.vertAccLim                                       = 3.5    ;%TECS_VERT_ACC
AP_TECS.spdCompFiltOmega                                 = 2      ;%TECS_SPD_OMEGA 
AP_TECS.p_ff_throttle                                    = 0.5    ;%TECS_P_FF_THR
AP_TECS.pitch_max                                        = 0      ;%TECS_PITCH_MAX
AP_TECS.pitch_min                                        = 0      ;%TECS_PITCH_MIN
AP_TECS.spdWeight                                        = 0.9    ;%TECS_SPDWEIGHT
AP_TECS.throttle_max                                     = 100    ;%TECS_THR_MAX 
AP_TECS.throttle_min                                     = 0      ;%TECS_THR_MIN                                                                   
AP_L1.L1_damping                                         = 0.75   ;%NAVL1_DAMPING
AP_L1.L1_period                                          = 15     ;%NAVL1_PERIOD
AP_L1.L1_xtrack_i_gain                                   = 0      ;%NAVL1_XTRACK_I 
AP_L1.loiter_bank_limit                                  = 0      ;%NAVL1_LIM_BANK
Plane.airspeed_max                                       =  22    ;%ARSPD_MAX
Plane.airspeed_min                                       =  15    ;%ARSPD_MIN
Plane.kff_rudder_mix                                     =  1.2   ;%KFF_RDDRMIX
Plane.scaling_speed                                      =  17    ;%SCALING_SPEED
Plane.pitch_limit_max_cd                                 = 2000   ;%LIM_PITCH_MAX
Plane.pitch_limit_min_cd                                 = -1500  ;%LIM_PITCH_MIN
Copter_Plane.aspeed_cp                                   = 30     ;%ARSPD_CP
Copter_Plane.aspeed_c2p                                  = 8      ;%ARSPD_C2P
Copter_Plane.aspeed_c2ps                                 = 18     ;%ARSPD_C2PS
Copter_Plane.p_plane_c2p                                 = 0.6    ;%P_PLANE_C2P
Copter_Plane.p_plane_cp                                  = 0.4    ;%P_PLANE_CP
Copter_Plane.p_k_elevator_c2p                            = 2      ;%P_KE_C2P
Copter_Plane.p_tail_tilt                                 = 3      ;%P_TAIL_TILT
Copter_Plane.pitch_target_c2p                            = 0      ;%PITCH_TAR_C2P
Copter_Plane.tail_tilt_c2p                               = -2800  ;%TAIL_TILT_C2P
Copter_Plane.k_throttle_c2p                              = 0.4    ;%K_THR_C2P 
Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p                = 0.2    ;%PSC_ACCZ_FIL_C2P
Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ                    = 4      ;%ARSPD_CP
Copter_Plane.thr_out_min_c2p                             = 0.5    ;%ARSPD_C2P
Copter_Plane.yaw_max_c2p                                 = 0.1    ;%ARSPD_C2PS
Copter_Plane.p_tilt_pitch_target                         = 0.4    ;%P_PLANE_C2P
Copter_Plane.tail_tilt_p2c                               = -1300  ;%P_PLANE_CP
Copter_Plane.p_plane_p2c                                 = 1      ;%P_KE_C2P
Copter_Plane.pitch_target_p2c                            = 500    ;%PITCH_TAR_C2P
Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c         = 0.05   ;%PSC_ACCZ_FLTP2C
Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ             = 3      ;%PSC_ACCZ_FLT
Copter_Plane.Fix2Rotor_delay_s                           = 0.8    ;%P2C_DELAY_S
Copter_Plane.tail_tilt_rate                              = 1500   ;%TAIL_TILT_RATE
Copter_Plane.heading_hold                                = 0      ;%HEADING_HOLD       
Copter_Plane.spdWeight                                   = 0.9    ;%SPDWEINGHT       
Copter_Plane.roll_limit_cd                               = 4000   ;%RLL_LIMIT_CD       
Copter_Plane.EAS_dem_cm                                  = 2000   ;%EAS_DEM_CM       
Copter_Plane.L1_radius                                   = 60     ;%L1_RADIUS                                 
Copter_Plane.radius                                      = 80     ;%RADIUS                                 
Copter_Plane.loiter_direction                            = 1      ;%LOITER_DIR                                 
Copter_Plane.dist_min                                    = 50     ;%DIST_MIN                                 
Copter_Plane.k_flap_TakeOff                              = -3500  ;%K_FLAP_TAKEOFF                                  
Copter_Plane.k_flap_Land                                 = 3500   ;%K_FLAP_LAND                                 
Copter_Plane.thr_out_min                                 = 0.4    ;%THR_OUT_MIN                                 
Copter_Plane.throttle_ground                             = 0.45   ;%THR_GROUND                                 
Copter_Plane.throttle_off_rate                           = 0.03   ;%THR_OFF_RATE                                 
Copter_Plane.weathervane_gain                            = 1.2    ;%WVANE_GAIN                                
Copter_Plane.weathervane_min_roll                        = 4      ;%WVANE_MINROLL                                 
Copter_Plane.yaw_rate_max                                = 50     ;%YAW_RATE_MAX                                 
Copter_Plane.vel_forward_gain                            = 1      ;%VFWD_GAIN                                  
Copter_Plane.vel_forward_min_pitch                       = -4     ;%VFWD_MIN_PIT                                  
Copter_Plane.vel_forward_tail_tilt_max                   = 2000   ;%VFWD_MAX_TAIL                                  
Copter_Plane.arspeed_filt                                = 5      ;%ARSPD_FILT                                  
 
 
AP_Motors.Kx                                             = 0                            ;
AP_Motors.thrust_slew_time                               = 0.3                          ;
AP_Motors.thr_mix_min                                    = 0.1                          ;
AP_Motors.thr_mix_max                                    = 0.5                          ;
AP_Motors.current_tilt                                   = 0                            ;
AP_Motors.yaw_headroom                                   = 0                            ;
AP_Motors.thrust_curve_expo                              = 0.25                         ;
AP_Motors.spin_max                                       = 0.95                         ;
AP_Motors.batt_voltage_max                               = 0                            ;
AP_Motors.batt_voltage_min                               = 0                            ;
AP_Motors.batt_current_max                               = 0                            ;
AP_Motors.pwm_max                                        = 2000                         ;
AP_Motors.pwm_min                                        = 1000                         ;
AP_Motors.spin_min                                       = 0.15                         ;
AP_Motors.spin_arm                                       = 0.1                          ;
AP_Motors.batt_current_time_constant                     = 5                            ;
AP_Motors.throttle_hover                                 = 0.4                         ;
AP_Motors.disarm_disable_pwm                             = 1                            ;
AP_Motors.spool_up_time                                  = 0.5                          ;
AP_Motors.slew_up_time                                   = 0                            ;%0.5
AP_Motors.slew_dn_time                                   = 0                            ;%0.5
AP_Motors.safe_time                                      = 1                            ;
AP_Motors.angle_limit_tc                                 = 1                            ;
AP_Motors.angle_boost_enabled                            = 1                            ;
AP_Motors.air_density_ratio                              = 1                            ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%设置参数
AC_Attitude.use_sqrt_controller                          = 1;
AC_Attitude.rate_bf_ff_enabled                           = 1;
AC_Attitude.accel_roll_max                               = 72000;
AC_Attitude.accel_pitch_max                              = 36000;
AC_Attitude.accel_yaw_max                                = 18000;
AC_Attitude.ang_vel_roll_max                             = 360;
AC_Attitude.ang_vel_pitch_max                            = 360;
AC_Attitude.ang_vel_yaw_max                              = 360;

AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS    = radians(40.0) ;    %minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS    = radians(720.0);    %maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS     = radians(10.0) ;    % minimum body-frame acceleration limit for the stability controller (for yaw axis)
AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS     = radians(120.0) ;   % maximum body-frame acceleration limit for the stability controller (for yaw axis)
AC_Attitude.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX          = 0.8;
AC_Attitude.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN                   = 10 ;
AC_Attitude.AC_ATTITUDE_THRUST_ERROR_ANGLE               = radians(30.0);               %Thrust angle error above which yaw corrections are limited;

AC_PosControl.speed_down_cms                             = -300;%POSCONTROL_SPEED_DOWN
AC_PosControl.speed_up_cms                               = 300;%POSCONTROL_SPEED_UP
AC_PosControl.speed_cms                                  = 300;%POSCONTROL_SPEED
AC_PosControl.accel_z_cms                                = 100;%POSCONTROL_ACCEL_Z
AC_PosControl.accel_cms                                  = 100;%POSCONTROL_ACCEL_XY
AC_PosControl.leash                                      = 100;%POSCONTROL_LEASH_LENGTH_MIN
AC_PosControl.leash_down_z                               = 100;%POSCONTROL_LEASH_LENGTH_MIN
AC_PosControl.leash_up_z                                 = 100;%POSCONTROL_LEASH_LENGTH_MIN
AC_PosControl.accel_xy_filt_hz                           = 10;%POSCONTROL_ACCEL_FILTER_HZ
AC_PosControl.vibe_comp_enabled                          = 1;
AC_PosControl.accel_last_z_cms                           = 0;%

AC_PosControl.POSCONTROL_VEL_ERROR_CUTOFF_FREQ           = 1.5;
AC_PosControl.POSCONTROL_VIBE_COMP_I_GAIN                = 0.125;
AC_PosControl.POSCONTROL_VIBE_COMP_P_GAIN                = 0.25;
AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ            = 10;
AC_PosControl.POSCONTROL_ACCEL_XY_MAX                    = 357;
AC_PosControl.POSCONTROL_OVERSPEED_GAIN_Z                = 2;%
AC_PosControl.POSCONTROL_JERK_RATIO                      = 1.0;%
AC_PosControl.POSCONTROL_ACCELERATION_MIN                = 50.0;% minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
AC_PosControl.POSCONTROL_LEASH_LENGTH_MIN                = 100.0;%minimum leash lengths in cm
AC_PosControl.thr_out_min                                = 0;

AP_Motors.LAND_CHECK_ANGLE_ERROR_DEG                     = 30;% maximum angle error to be considered landing
AP_Motors.LAND_CHECK_LARGE_ANGLE_CD                      = 15;% maximum angle target to be considered landing
AP_Motors.LAND_CHECK_ACCEL_MOVING                        = 3.0;% maximum acceleration after subtracting gravity
AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CW                 = -1;
AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CCW                = 1;
AP_Motors.AC_ATTITUDE_CONTROL_MAX                        = 5;
AP_Motors.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX   = 0.8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SINS.accel_x                                             = 0;
SINS.accel_y                                             = 0;
SINS.accel_z                                             = 0;
[SINS.gyro_x ,SINS.gyro_y ,SINS.gyro_z]                  = deal(0,0,0);
[SINS.roll ,SINS.pitch ,SINS.yaw]                        = deal(0,0,0);
SINS.curr_vel                                            = [0 0 0];
SINS.curr_loc                                            = [0 0];
SINS.curr_alt                                            = 0 ;
SINS.aspeed                                              = 0;
SINS.EAS2TAS                                             = 1;
SINS.groundspeed_vector                                  = [0 0];
SINS.curr_pos                                            = [0 0];
SINS.z_accel_meas                                        = 0;
SINS.rot_body_to_ned                                     = eye(3,3);

%中间变量
AC_rate_roll_pid.flags_reset_filter                      = 1;
AC_rate_roll_pid.disable_integrator                      = 0;
AC_rate_roll_pid.target                                  = 0;
AC_rate_roll_pid.error                                   = 0;
AC_rate_roll_pid.error_last                              = 0;
AC_rate_roll_pid.integrator                              = 0;
AC_rate_roll_pid.derivative                              = 0;
AC_rate_roll_pid.slew_amplitude                          = 0;
AC_rate_roll_pid.slew_filterg                            = 0;
AC_rate_roll_pid.last_sample                             = 0;
AC_rate_roll_pid.Dmod                                    = 0;

AC_rate_pitch_pid.flags_reset_filter                     = 1;
AC_rate_pitch_pid.disable_integrator                     = 0;
AC_rate_pitch_pid.target                                 = 0;
AC_rate_pitch_pid.error                                  = 0;
AC_rate_pitch_pid.error_last                             = 0;
AC_rate_pitch_pid.integrator                             = 0;
AC_rate_pitch_pid.derivative                             = 0;
AC_rate_pitch_pid.slew_amplitude                         = 0;
AC_rate_pitch_pid.slew_filterg                           = 0;
AC_rate_pitch_pid.last_sample                            = 0;
AC_rate_pitch_pid.Dmod                                   = 0;

AC_rate_yaw_pid.flags_reset_filter                       = 1;
AC_rate_yaw_pid.disable_integrator                       = 0;
AC_rate_yaw_pid.target                                   = 0;
AC_rate_yaw_pid.error                                    = 0;
AC_rate_yaw_pid.error_last                               = 0;
AC_rate_yaw_pid.integrator                               = 0;
AC_rate_yaw_pid.derivative                               = 0;
AC_rate_yaw_pid.slew_amplitude                           = 0;
AC_rate_yaw_pid.slew_filterg                             = 0;
AC_rate_yaw_pid.last_sample                              = 0;
AC_rate_yaw_pid.Dmod                                     = 0;

AC_Attitude.thrust_error_angle                           = 0;
AC_Attitude.rate_target_ang_vel                          = [0 0 0];
AC_Attitude.attitude_ang_error                           = [1 0 0 0];
AC_Attitude.attitude_error_vector                        = [0 0 0];
AC_Attitude.attitude_target_quat                         = [1 0 0 0];
AC_Attitude.attitude_target_euler_angle                  = [0 0 0];
AC_Attitude.attitude_target_euler_rate                   = [0 0 0];
AC_Attitude.attitude_target_ang_vel                      = [0 0 0];
AC_Attitude.althold_lean_angle_max                       = 0;

AC_PosControl.flags_recalc_leash_z                       = true;
AC_PosControl.flags_recalc_leash_xy                      = true;
AC_PosControl.flags_reset_desired_vel_to_pos             = true;
AC_PosControl.flags_reset_accel_to_lean_xy               = true;
AC_PosControl.flags_reset_rate_to_accel_z                = true;
AC_PosControl.flags_freeze_ff_z                          = true;
AC_PosControl.flags_use_desvel_ff_z                      = true;
AC_PosControl.flags_vehicle_horiz_vel_override           = false;

AC_PosControl.limit_pos_up                               = true;
AC_PosControl.limit_pos_down                             = true;
AC_PosControl.limit_vel_up                               = true;
AC_PosControl.limit_vel_down                             = true;
AC_PosControl.limit_accel_xy                             = true;

AC_PosControl.pos_error                                  = [0 0 0];
AC_PosControl.pos_target                                 = [0 0 0];
AC_PosControl.vel_target                                 = [0 0 0];
AC_PosControl.accel_target                               = [0 0 0];
AC_PosControl.accel_target_filter                        = [0 0 0];
AC_PosControl.roll_target                                = 0;
AC_PosControl.pitch_target                               = 0;
AC_PosControl.target_yaw_rate                            = 0;
AC_PosControl.vehicle_horiz_vel                          = [0 0];
AC_PosControl.vel_desired                                = [0 0 0];
AC_PosControl.vel_error_filter                           = [0 0 0];
AC_PosControl.vel_error                                  = [0 0 0];
AC_PosControl.vel_last                                   = [0 0 0];
AC_PosControl.accel_desired                              = [0 0 0];
AC_PosControl.vel_z_control_ratio                        = 0;

AC_PosControl.pid_vel_xy.flags_reset_filter              = 0;
AC_PosControl.pid_vel_xy.disable_integrator              = true;
AC_PosControl.pid_vel_xy.target                          = [0 0];
AC_PosControl.pid_vel_xy.error                           = [0 0];
AC_PosControl.pid_vel_xy.error_last                      = [0 0];
AC_PosControl.pid_vel_xy.integrator                      = [0 0];
AC_PosControl.pid_vel_xy.derivative                      = [0 0];
%  AC_PosControl.pid_vel_xy.slew_amplitude                = [0 0];
%  AC_PosControl.pid_vel_xy.slew_filterg                  = [0 0];
%  AC_PosControl.pid_vel_xy.last_sample                   = [0 0];
%  AC_PosControl.pid_vel_xy.Dmod                          = [1 1];

AC_PosControl.pid_accel_z.flags_reset_filter             = 0;
AC_PosControl.pid_accel_z.disable_integrator             = true;
AC_PosControl.pid_accel_z.target                         = 0;
AC_PosControl.pid_accel_z.error                          = 0;
AC_PosControl.pid_accel_z.error_last                     = 0;
AC_PosControl.pid_accel_z.integrator                     = 0;
AC_PosControl.pid_accel_z.derivative                     = 0;
AC_PosControl.pid_accel_z.slew_amplitude                 = 0;
AC_PosControl.pid_accel_z.slew_filterg                   = 0;
AC_PosControl.pid_accel_z.last_sample                    = 0;
AC_PosControl.pid_accel_z.pid_info_P                     = 0;
AC_PosControl.pid_accel_z.pid_info_D                     = 0;
AC_PosControl.pid_accel_z.Dmod                           = 1;

AP_rate_roll.last_out                                    = 0;
AP_rate_roll.pid_info_target                             = 0;
AP_rate_roll.pid_info_actual                             = 0;
AP_rate_roll.pid_info_error                              = 0;
AP_rate_roll.pid_info_P                                  = 0;
AP_rate_roll.pid_info_I                                  = 0;
AP_rate_roll.pid_info_D                                  = 0;
AP_rate_roll.pid_info_FF                                 = 0;
AP_rate_roll.last_pid_info_D                             = 0;

AP_rate_roll.slew_filterg                                = 0;
AP_rate_roll.slew_rate_amplitude                         = 0;
AP_rate_roll.D_gain_modifier                             = 0;
AP_rate_roll.pid_info_Dmod                               = 0;

AP_rate_pitch.last_out                                   = 0;
AP_rate_pitch.pid_info_target                            = 0;
AP_rate_pitch.pid_info_actual                            = 0;
AP_rate_pitch.pid_info_error                             = 0;
AP_rate_pitch.pid_info_P                                 = 0;
AP_rate_pitch.pid_info_I                                 = 0;
AP_rate_pitch.pid_info_D                                 = 0;
AP_rate_pitch.pid_info_FF                                = 0;
AP_rate_pitch.last_pid_info_D                            = 0;

AP_rate_pitch.slew_filterg                               = 0;
AP_rate_pitch.slew_rate_amplitude                        = 0;
AP_rate_pitch.D_gain_modifier                            = 0;
AP_rate_pitch.pid_info_Dmod                              = 0;

AP_rate_yaw.K_D_last                                     = 0;
AP_rate_yaw.pid_info_I                                   = 0;
AP_rate_yaw.pid_info_D                                   = 0;
AP_rate_yaw.last_rate_hp_out                             = 0;
AP_rate_yaw.last_rate_hp_in                              = 0;
AP_rate_yaw.integrator                                   = 0;
AP_rate_yaw.last_out                                     = 0;

AP_TECS.THRmaxf                                          = 1;
AP_TECS.THRminf                                          = 0;
AP_TECS.throttle_dem                                     = 0;
AP_TECS.last_throttle_dem                                = 0;
AP_TECS.integTHR_state                                   = 0;
AP_TECS.ff_throttle                                      = 0;

AP_TECS.hgt_dem                                          = 0;
AP_TECS.hgt_dem_in_old                                   = 0;
AP_TECS.hgt_dem_prev                                     = 0;
AP_TECS.hgt_dem_adj                                      = 0;
AP_TECS.hgt_dem_adj_last                                 = 0;
AP_TECS.hgt_rate_dem                                     = 0;

AP_TECS.pitch_dem                                        = 0;
AP_TECS.pitch_dem_unc                                    = 0;
AP_TECS.last_pitch_dem                                   = 0;
AP_TECS.PITCHmaxf                                        = 0;
AP_TECS.PITCHminf                                        = 0;
AP_TECS.pitch_max_limit                                  = 0;

AP_TECS.vel_dot                                          = 0;
AP_TECS.vdot_filter                                      = [0 0 0 0 0];
AP_TECS.climb_rate                                       = 0;
AP_TECS.EAS_dem                                          = 19;
AP_TECS.TAS_dem                                          = 19;
AP_TECS.TAS_state                                        = 19;
AP_TECS.TAS_dem_adj                                      = 19;
AP_TECS.TAS_rate_dem                                     = 0;
AP_TECS.integDTAS_state                                  = 0;
AP_TECS.TASmax                                           = 23;
AP_TECS.TASmin                                           = 15;

AP_TECS.SPE_dem                                          = 0;
AP_TECS.SPE_est                                          = 0;
AP_TECS.SPEdot_dem                                       = 0;
AP_TECS.SPEdot                                           = 0;
AP_TECS.SKE_dem                                          = 0;
AP_TECS.SKE_est                                          = 0;
AP_TECS.SKEdot_dem                                       = 0;
AP_TECS.SKEdot                                           = 0;
AP_TECS.STE_error                                        = 0;
AP_TECS.STEdot_min                                       = 0;
AP_TECS.STEdot_max                                       = 0;
AP_TECS.STEdotErrLast                                    = 0;
AP_TECS.integSEB_state                                   = 0;



AP_L1.target_bearing_cd                                  = 0;
AP_L1.L1_dist                                            = 0;
AP_L1.crosstrack_error                                   = 0;
AP_L1.nav_bearing                                        = 0;
AP_L1.L1_xtrack_i_gain_prev                              = 0;
AP_L1.L1_xtrack_i                                        = 0;
AP_L1.last_Nu                                            = 0;
AP_L1.latAccDem                                          = 0;
AP_L1.WPcircle                                           = 0;
AP_L1.bearing_error                                      = 0;
AP_L1.data_is_stale                                      = 0;
AP_L1.mode_L1                                            = 0;
AP_L1.reverse                                            = 0;

Plane.highest_airspeed                                   = 0;
Plane.nav_pitch_cd                                       = 0;
Plane.kff_throttle_to_pitch                              = 0;
Plane.inverted_flight                                    = 0;
Plane.smoothed_airspeed                                  = 0;
Plane.aerodynamic_load_factor                            = 0;
Plane.nav_roll_cd                                        = 0;
Plane.roll_limit_cd                                      = 0;

Copter_Plane.prev_WP                                      = [40,100]                           ;
Copter_Plane.next_WP                                      = [40,100.01]                        ;
Copter_Plane.center_WP                                    = [40,100]                           ;
Copter_Plane.loc_origin                                   = [40,100]                           ;
Copter_Plane.loc                                          = loc                                ;
Copter_Plane.inint_hgt                                    = 0                                      ;
Copter_Plane.armed                                       = 1            ;
Copter_Plane.Mode                                        =  ENUM_Mode.MANUAL                ;
Copter_Plane.State                                       =  ENUM_State.Copter;
Copter_Plane.hgt_dem_cm                                  = 0            ;
Copter_Plane.roll_target_pilot                           = 0            ;
Copter_Plane.pitch_target_pilot                          = 0            ;
Copter_Plane.climb_rate_cms                              = 0            ;
Copter_Plane.take_off_land                                = 0                                      ;
Copter_Plane.weathervane_last_output                      = 0                                      ;
Copter_Plane.vel_forward_integrator                       = 0                                       ;
Copter_Plane.arspeed_temp                                 = 0                                       ;
Copter_Plane.disable_AP_roll_integrator                   = 0                                       ;
Copter_Plane.disable_AP_pitch_integrator                  = 0                                       ;
Copter_Plane.disable_AP_yaw_integrator                    = 0                                       ;
Copter_Plane.disable_AP_rate_roll_gains_D                 = 0                                       ;
Copter_Plane.disable_AP_rate_pitch_roll_ff                = 0                                       ;
Copter_Plane.disable_AP_rate_pitch_gains_D                = 0                                       ;
Copter_Plane.disable_AP_rate_yaw_K_FF                     = 0                                       ;



AP_Motors.batt_current                                   = 0                            ;
AP_Motors.batt_resistance                                = 0                            ;
AP_Motors.batt_voltage                                   = 0                            ;
AP_Motors.batt_voltage_resting_estimate                  = 0                            ;
AP_Motors.batt_voltage_filt                              = 0                            ;
AP_Motors.lift_max                                       = 1                            ;
AP_Motors.spool_desired                                  = DesiredSpoolState.SHUT_DOWN  ;
AP_Motors.spool_state                                    = SpoolState.SHUT_DOWN         ;
AP_Motors.angle_boost                                    = 0                            ;

AP_Motors.roll_in                                        = 0                            ;
AP_Motors.pitch_in                                       = 0                            ;
AP_Motors.yaw_in                                         = 0                            ;

AP_Motors.roll_factor                                    = [0 0 0 0]                            ;
AP_Motors.pitch_factor                                   = [0 0 0 0]                            ;
AP_Motors.yaw_factor                                     = [0 0 0 0]                            ;
AP_Motors.actuator                                       = [0 0 0 0]                    ;
AP_Motors.thrust_rpyt_out                                = [0 0 0 0]                    ;

AP_Motors.thrust_boost                                   = 0;
AP_Motors.thrust_boost_ratio                             = 0;
AP_Motors.thrust_balanced                                = 0;
AP_Motors.disarm_safe_timer                              = 0;
AP_Motors.spin_up_ratio                                  = 0;
AP_Motors.throttle_cutoff_frequency                      = 1;
AP_Motors.throttle_limit                                 = 1;
AP_Motors.throttle_avg_max                               = 0;
AP_Motors.throttle_rpy_mix                               = 0;
AP_Motors.throttle_rpy_mix_desired                       = 0;
AP_Motors.throttle_thrust_max                            = 1;
AP_Motors.throttle_out                                   = 0;
AP_Motors.thrust_rpyt_out_filt                           = [0 0 0 0];
AP_Motors.land_accel_ef_filter                           = 0;
AP_Motors.motor_lost_index                               = 1;

AP_Motors.limit_roll                                     = false;
AP_Motors.limit_pitch                                    = false;
AP_Motors.limit_yaw                                      = false;
AP_Motors.limit_throttle_lower                           = false;
AP_Motors.limit_throttle_upper                           = false;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SlewLimiter.pos_event_stored               = false                ;
SlewLimiter.neg_event_stored               = false                ;
SlewLimiter.pos_event_index                = 1                 ;
SlewLimiter.neg_event_index                = 1                 ;
SlewLimiter.pos_event_ms                   = [0 0]                    ;
SlewLimiter.neg_event_ms                   = [0 0]                    ;
SlewLimiter.max_pos_slew_rate              = 0               ;
SlewLimiter.max_neg_slew_rate              = 0               ;
SlewLimiter.modifier_slew_rate             = 0              ;
SlewLimiter.output_slew_rate               = 0                ;
SlewLimiter.last_sample                    = 0                     ;
SlewLimiter.slew_filter                    = 0                     ;
SlewLimiter.N_EVENTS                       = 2                        ;
SlewLimiter.max_pos_slew_event_ms          = 0           ;
SlewLimiter.max_neg_slew_event_ms          = 0           ;
SlewLimiter.MODIFIER_GAIN                  = 1.5                   ;
SlewLimiter.WINDOW_MS                      = 100                       ;
SlewLimiter.DERIVATIVE_CUTOFF_FREQ         = 25          ;
SlewLimiter.slew_rate_max                  = 3000                   ;
SlewLimiter.slew_rate_tau                  = 0.3                   ;

