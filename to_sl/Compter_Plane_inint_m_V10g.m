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
%sl
    POSCONTROL_ACC_Z_DT=0.0025;    % vertical acceleration controller dt default
    gains_rmax_roll=0;   
    thrust_slew_time=0.3;%%%%%%%%%%%%%%%%油门时间
    throttle_slewrate=100;%%%%%%%%%%%%%%%%TECS 油门变化率
    yaw_rate_max=50;
    plane_mode=ENUM_plane_mode.V10;
    pwm_tail=1000;
    vel_forward_integrator=0;
    
 p_angle_pitch                          = 3                            ;
 p_angle_roll                           = 3.3                          ;
 p_angle_yaw                            = 3                            ;
 ATC_RAT_PIT_D                          = 0                            ;
 ATC_RAT_PIT_FF                         = 0                            ;
 ATC_RAT_PIT_FILT                       = 20                           ;
 ATC_RAT_PIT_I_inint                    = 0.1                          ;
 ATC_RAT_PIT_IMAX                       = 0.25                         ;
 ATC_RAT_PIT_P                          = 0.2                          ;
 ATC_RAT_RLL_D                          = 0                            ;
 ATC_RAT_RLL_FF                         = 0                            ;
 ATC_RAT_RLL_FILT                       = 20                           ;
 ATC_RAT_RLL_I_inint                    = 0.1                          ;
 ATC_RAT_RLL_IMAX                       = 0.25                         ;
 ATC_RAT_RLL_P                          = 0.28                         ;
 ATC_RAT_YAW_D                          = 0                            ;
 ATC_RAT_YAW_FF                         = 0                            ;
 ATC_RAT_YAW_FILT                       = 5                            ;
 ATC_RAT_YAW_I_inint                    = 0                            ;
 ATC_RAT_YAW_IMAX                       = 0                            ;
 ATC_RAT_YAW_P                          = 0.19                         ;
 POSCONTROL_POS_Z_P                     = 1                            ;
 POSCONTROL_VEL_Z_P                     = 1                            ;
 POSCONTROL_ACC_Z_P                     = 0.9                          ;
 POSCONTROL_ACC_Z_I_inint               = 0.18                         ;
 POSCONTROL_ACC_Z_D                     = 0.3                          ;
 POSCONTROL_ACC_Z_IMAX                  = 200                          ;
 POSCONTROL_ACC_Z_FILT_HZ_inint         = 4                            ;
 POSCONTROL_THROTTLE_CUTOFF_FREQ_inint  = 3                            ;
 gains_tau_pitch                        = 1                            ;
 gains_P_pitch                          = 1.2                          ;
 gains_D_pitch_inint                    = 0.45                         ;
 gains_I_pitch                          = 0.20                         ;
 roll_ff_pitch_inint                    = 0.85                         ;
 gains_imax_pitch                       = 3000                         ;
 gains_tau_roll                         = 0.5                          ;
 gains_P_roll                           = 1.27                         ;
 gains_D_roll_inint                     = 0.22                         ;
 gains_I_roll                           = 0                            ;
 gains_imax_roll                        = 3000                         ;
 gains_FF_roll                          = 0                            ;
 K_A_yaw                                = 0.25                         ;
 K_I_yaw                                = 0.15                         ;
 K_D_yaw                                = 0.35                         ;
 K_FF_yaw_inint                         = 1                            ;
 imax_yaw                               = 1500                         ;
 current_tilt                           = 0                           ;
 throttle_hover                         = 0.25                          ;
 maxClimbRate                           = 5                            ;
 minSinkRate                            = 3                            ;
 timeConstant                           = 3                            ;
 thrDamp                                = 0.7                          ;
 integGain                              = 0.25                         ;
 vertAccLim                             = 3.5                          ;
 spdCompFiltOmega                       = 2                            ;
 rollComp                               = 20                           ;
 spdWeight_inint                        = 0.9                          ;
 ptchDamp                               = 0.7                          ;
 maxSinkRate                            = 7                            ;
 throttle_cruise                        = 40                           ;
 arspeed_filt                           = 5                            ;
 EAS_dem_cm                             = 2000                         ;
 POSCONTROL_POS_XY_P                    = 1.1                          ;
 POSCONTROL_VEL_XY_P                    = 1.1                          ;
 POSCONTROL_VEL_XY_I_inint              = 0.15                         ;
 POSCONTROL_VEL_XY_D                    = 0.12                         ;
 POSCONTROL_VEL_XY_IMAX                 = 1000                         ;
 POSCONTROL_VEL_XY_FILT_HZ              = 2                            ;
 POSCONTROL_VEL_XY_FILT_D_HZ            = 5                            ;
 L1_period                              = 15                           ;
 L1_damping                             = 0.75                         ;
 L1_xtrack_i_gain                       = 0                            ;
 p_tail_tilt                            = 3                            ;
 p_tilt_pitch_target                    = 0.4                          ;
 aspeed_c2p                             = 8                            ;
 p_plane_c2p                            = 0.6                          ;
 yaw_max_c2p                            = 0.1                          ;
 weathervane_min_roll                   = 4                            ;
 weathervane_gain                       = 1.2                          ;
 POSCONTROL_ACC_Z_FILT_HZ_c2p           = 0.2                          ;
 kff_rudder_mix                         = 1.2                          ;
 radius                                 = 80                           ;
 loiter_direction                       = 1                            ;
 tail_tilt_c2p                          = -2800                        ;
 tail_tilt_p2c                          = -1300                        ;
 tail_tilt_rate                         = 1500                         ;
 aspeed_c2ps                            = 18                           ;
 POSCONTROL_ACCEL_FILTER_HZ             = 10                           ;
 POSCONTROL_VEL_ERROR_CUTOFF_FREQ       = 1.5                          ;
 roll_limit_cd_inint                    = 4000                         ;
 cutoff_freq_a                          = 25                           ;
 cutoff_freq_g                          = 35                           ; 
 p_ff_throttle                          = 0.5                          ;
 vel_forward_gain                       = 1                            ;
 vel_forward_min_pitch                  = -4                           ; 
 aspeed_cp                              = 30                           ;
 p_plane_cp                             = 0.4                          ;
 POSCONTROL_SPEED                       = 300                          ;
 POSCONTROL_ACCEL_XY                    = 100                          ;
 POSCONTROL_ACCEL_Z                     = 100                          ;
 thr_out_min_inint                      = 0.4                           ;       
 pitch_target_p2c                       = 500                          ;
 k_throttle_c2p                         = 0.4                          ;
 throttle_off_rate                      = 0.03                         ;
 throttle_ground                        = 0.45                         ;
 yaw_in_max                             = 1                            ;
 pitch_target_c2p                       = 0                            ;
 airspeed_max                           = 22                           ;
 airspeed_min 							          	= 15                           ;
 p_plane_p2c                            = 1                              ; 
 POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c    = 0.05                          ;   
 Fix2Rotor_delay_s                      = 0.8                          ;
 p_k_elevator_c2p                       = 2                            ;
 thr_out_min_c2p                        = 0.5                          ;   
 heading_hold                           = 0                            ;
 k_flap_TakeOff                         = -3500                        ;
 k_flap_Land                            = 3500                         ;
 p_flap_plane                           = 0                            ;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 ATC_RAT_PIT_D                          = 0                            ;
 ATC_RAT_PIT_FF                         = 0                            ;
 ATC_RAT_PIT_FILT                       = 20                           ;
 ATC_RAT_PIT_I_inint                    = 0.1                          ;
 ATC_RAT_PIT_IMAX                       = 0.25                         ;
 ATC_RAT_PIT_P                          = 0.2                          ;
 
 ATC_RAT_RLL_D                          = 0                            ;
 ATC_RAT_RLL_FF                         = 0                            ;
 ATC_RAT_RLL_FILT                       = 20                           ;
 ATC_RAT_RLL_I_inint                    = 0.1                          ;
 ATC_RAT_RLL_IMAX                       = 0.25                         ;
 ATC_RAT_RLL_P                          = 0.28                         ;
 
 ATC_RAT_YAW_D                          = 0                            ;
 ATC_RAT_YAW_FF                         = 0                            ;
 ATC_RAT_YAW_FILT                       = 5                            ;
 ATC_RAT_YAW_I_inint                    = 0                            ;
 ATC_RAT_YAW_IMAX                       = 0                            ;
 ATC_RAT_YAW_P                          = 0.19                         ;
 
 rate_yaw_pid_reset_filter=0;
 rate_yaw_pid_input=0;
 rate_yaw_pid_derivative=0;
 rate_yaw_pid_integrator=0;
 motors_limit_yaw=0;
 
 rate_roll_pid_reset_filter=0;
 rate_roll_pid_input=0;
 rate_roll_pid_derivative=0;
 rate_roll_pid_integrator=0;
 motors_limit_roll_pitch=0;
 
 rate_pitch_pid_reset_filter=0;
 rate_pitch_pid_input=0;
 rate_pitch_pid_derivative=0;
 rate_pitch_pid_integrator=0;
 
 rate_roll_pid.kp                      =0.2;
 rate_roll_pid.ki                      =0.1;
 rate_roll_pid.kd                      =0;
 rate_roll_pid.kff                     =0;
 rate_roll_pid.kimax                   =0.25;
 rate_roll_pid.filt_T_hz               =20;
 rate_roll_pid.filt_E_hz               =20;
 rate_roll_pid.filt_D_hz               =20;
 rate_roll_pid.slew_rate_max           =0;
 rate_roll_pid.slew_rate_tau           =0.1;
 
 rate_pitch_pid.kp                      =0.28;
 rate_pitch_pid.ki                      =0.1;
 rate_pitch_pid.kd                      =0;
 rate_pitch_pid.kff                     =0;
 rate_pitch_pid.kimax                   =0.25;
 rate_pitch_pid.filt_T_hz               =20;
 rate_pitch_pid.filt_E_hz               =20;
 rate_pitch_pid.filt_D_hz               =20;
 rate_pitch_pid.slew_rate_max           =0;
 rate_pitch_pid.slew_rate_tau           =0.1;
 
 rate_yaw_pid.kp                      =0.19;
 rate_yaw_pid.ki                      =0.0;
 rate_yaw_pid.kd                      =0;
 rate_yaw_pid.kff                     =0;
 rate_yaw_pid.kimax                   =0.0;
 rate_yaw_pid.filt_T_hz               =20;
 rate_yaw_pid.filt_E_hz               =20;
 rate_yaw_pid.filt_D_hz               =20;
 rate_yaw_pid.slew_rate_max           =0;
 rate_yaw_pid.slew_rate_tau           =0.1;
 
 rate_roll_pid.flags_reset_filter     =1;
 rate_roll_pid.disable_integrator     =0;
 rate_roll_pid.target                 =0;
 rate_roll_pid.error                  =0;
 rate_roll_pid.error_last             =0;
 rate_roll_pid.integrator             =0;
 rate_roll_pid.derivative             =0;
 rate_roll_pid.slew_amplitude         =0;
 rate_roll_pid.slew_filterg           =0;
 rate_roll_pid.last_sample            =0;
 rate_roll_pid.Dmod                   =0;
 
 rate_pitch_pid.flags_reset_filter     =1;
 rate_pitch_pid.disable_integrator     =0;
 rate_pitch_pid.target                 =0;
 rate_pitch_pid.error                  =0;
 rate_pitch_pid.error_last             =0;
 rate_pitch_pid.integrator             =0;
 rate_pitch_pid.derivative             =0;
 rate_pitch_pid.slew_amplitude         =0;
 rate_pitch_pid.slew_filterg           =0;
 rate_pitch_pid.last_sample            =0;
 rate_pitch_pid.Dmod                   =0;
 
 rate_yaw_pid.flags_reset_filter     =1;
 rate_yaw_pid.disable_integrator     =0;
 rate_yaw_pid.target                 =0;
 rate_yaw_pid.error                  =0;
 rate_yaw_pid.error_last             =0;
 rate_yaw_pid.integrator             =0;
 rate_yaw_pid.derivative             =0;
 rate_yaw_pid.slew_amplitude         =0;
 rate_yaw_pid.slew_filterg           =0;
 rate_yaw_pid.last_sample            =0;
 rate_yaw_pid.Dmod                   =0;
 
 gains_tau_pitch                        = 1                            ;
 gains_P_pitch                          = 1.2                          ;
 gains_D_pitch_inint                    = 0.45                         ;
 gains_I_pitch                          = 0.20                         ;
 roll_ff_pitch_inint                    = 0.85                         ;
 gains_imax_pitch                       = 3000                         ;
 
 gains_tau_roll                         = 0.5                          ;
 gains_P_roll                           = 1.27                         ;
 gains_D_roll_inint                     = 0.22                         ;
 gains_I_roll                           = 0                            ;
 gains_imax_roll                        = 3000                         ;
 gains_FF_roll                          = 0                            ;
 
 K_A_yaw                                = 0.25                         ;
 K_I_yaw                                = 0.15                         ;
 K_D_yaw                                = 0.35                         ;
 K_FF_yaw_inint                         = 1                            ;
 imax_yaw                               = 1500                         ;
 
 AP_rate_roll.gains_tau                 =0.5;
 AP_rate_roll.gains_rmax                =0;
 AP_rate_roll.gains_P                   =1.27  ;
 AP_rate_roll.gains_D                   =0.22;
 AP_rate_roll.gains_I                   =0     ;
 AP_rate_roll.gains_FF                  =0;
 AP_rate_roll.gains_imax                =3000;
 AP_rate_roll.slew_rate_max             =0;
 AP_rate_roll.slew_rate_tau             =0.1;
 
 AP_rate_pitch.roll_ff                   =0.85;
 AP_rate_pitch.max_rate_neg              =0;
 AP_rate_pitch.gains_tau                 =1;
 AP_rate_pitch.gains_rmax                =0;
 AP_rate_pitch.gains_P                   =1.2  ;
 AP_rate_pitch.gains_D                   =0.45;
 AP_rate_pitch.gains_I                   =0.20     ;
 AP_rate_pitch.gains_FF                  =0;
 AP_rate_pitch.gains_imax                =3000;
 AP_rate_pitch.slew_rate_max             =0;
 AP_rate_pitch.slew_rate_tau             =0.1;
 
 AP_rate_yaw.K_A                        =0.25;
 AP_rate_yaw.K_I                        =0.15;
 AP_rate_yaw.K_D                        =0.35;
 AP_rate_yaw.K_FF                       =1;
 AP_rate_yaw.imax                       =1500;
 
 
AP_rate_roll.last_out                    = 0;
AP_rate_roll.pid_info_target             = 0;
AP_rate_roll.pid_info_actual             = 0;
AP_rate_roll.pid_info_error              = 0;
AP_rate_roll.pid_info_P                  = 0;
AP_rate_roll.pid_info_I                  = 0;
AP_rate_roll.pid_info_D                  = 0;
AP_rate_roll.pid_info_FF                 = 0;
AP_rate_roll.last_pid_info_D             = 0;

AP_rate_roll.slew_filterg                = 0;
AP_rate_roll.slew_rate_amplitude         = 0;
AP_rate_roll.D_gain_modifier             = 0;
AP_rate_roll.pid_info_Dmod               = 0;
 
AP_rate_pitch.last_out                    = 0;
AP_rate_pitch.pid_info_target             = 0;
AP_rate_pitch.pid_info_actual             = 0;
AP_rate_pitch.pid_info_error              = 0;
AP_rate_pitch.pid_info_P                  = 0;
AP_rate_pitch.pid_info_I                  = 0;
AP_rate_pitch.pid_info_D                  = 0;
AP_rate_pitch.pid_info_FF                 = 0;
AP_rate_pitch.last_pid_info_D             = 0;

AP_rate_pitch.slew_filterg                = 0;
AP_rate_pitch.slew_rate_amplitude         = 0;
AP_rate_pitch.D_gain_modifier             = 0;
AP_rate_pitch.pid_info_Dmod               = 0;
 
AP_rate_yaw.K_D_last                     = 0;
AP_rate_yaw.pid_info_I                   = 0;
AP_rate_yaw.pid_info_D                   = 0;
AP_rate_yaw.last_rate_hp_out             = 0;
AP_rate_yaw.last_rate_hp_in              = 0;
AP_rate_yaw.integrator                   = 0;
AP_rate_yaw.last_out                     = 0;
 

 AC_PosControl.p_pos_z                               = 1;
 AC_PosControl.p_vel_z                               = 1;
 AC_PosControl.pid_accel_z.kp                        = 0.9 ;%POSCONTROL_ACC_Z_P
 AC_PosControl.pid_accel_z.ki                        = 0.18;%POSCONTROL_ACC_Z_I
 AC_PosControl.pid_accel_z.kd                        = 0.3 ;%POSCONTROL_ACC_Z_D
 AC_PosControl.pid_accel_z.kff                       = 0  ;
 AC_PosControl.pid_accel_z.kimax                     = 200; %POSCONTROL_ACC_Z_IMAX
 AC_PosControl.pid_accel_z.filt_T_hz                 = 20;
 AC_PosControl.pid_accel_z.filt_E_hz                 = 4;%POSCONTROL_ACC_Z_FILT_HZ
 AC_PosControl.pid_accel_z.filt_D_hz                 = 20;
 AC_PosControl.pid_accel_z.slew_rate_max             = 0;
 AC_PosControl.pid_accel_z.slew_rate_tau             = 0.1;
 
 
 AC_PosControl.p_pos_xy                                = 1.1;
 AC_PosControl.pid_vel_xy.kp                           = 1.1;
 AC_PosControl.pid_vel_xy.ki                           = 0.15;
 AC_PosControl.pid_vel_xy.kd                           = 0.12;
 AC_PosControl.pid_vel_xy.kff                          = 0;
 AC_PosControl.pid_vel_xy.kimax                        = 1000;
 AC_PosControl.pid_vel_xy.filt_T_hz                    = 20;
 AC_PosControl.pid_vel_xy.filt_E_hz                    = 2;
 AC_PosControl.pid_vel_xy.filt_D_hz                    = 5;
%  AC_PosControl.pid_vel_xy.slew_rate_max                = 0;
%  AC_PosControl.pid_vel_xy.slew_rate_tau                = 0.1;
 
 AC_PosControl.speed_down_cms                          = -300;%POSCONTROL_SPEED_DOWN
 AC_PosControl.speed_up_cms                            = 300;%POSCONTROL_SPEED_UP
 AC_PosControl.speed_cms                               = 300;%POSCONTROL_SPEED
 AC_PosControl.accel_z_cms                             = 100;%POSCONTROL_ACCEL_Z
 AC_PosControl.accel_cms                               = 100;%POSCONTROL_ACCEL_XY
 AC_PosControl.leash                                   = 100;%POSCONTROL_LEASH_LENGTH_MIN
 AC_PosControl.leash_down_z                            = 100;%POSCONTROL_LEASH_LENGTH_MIN
 AC_PosControl.leash_up_z                              = 100;%POSCONTROL_LEASH_LENGTH_MIN
 AC_PosControl.accel_xy_filt_hz                        = 10;%POSCONTROL_ACCEL_FILTER_HZ
 AC_PosControl.vibe_comp_enabled                       = 1;
 AC_PosControl.accel_last_z_cms                        = 0;%a

 AC_PosControl.POSCONTROL_VEL_ERROR_CUTOFF_FREQ        = 1.5;
 AC_PosControl.POSCONTROL_VIBE_COMP_I_GAIN             = 0.125;
 AC_PosControl.POSCONTROL_VIBE_COMP_P_GAIN             = 0.25; 
 AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ         = 10;
 AC_PosControl.POSCONTROL_ACCEL_XY_MAX                 = 458;
 AC_PosControl.POSCONTROL_OVERSPEED_GAIN_Z             = 2;%a
 AC_PosControl.POSCONTROL_JERK_RATIO                   = 1.0;%a
 AC_PosControl.POSCONTROL_ACCELERATION_MIN             = 50.0;% minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
 AC_PosControl.POSCONTROL_LEASH_LENGTH_MIN             = 100.0;%minimum leash lengths in cm

 
 AC_PosControl.flags_recalc_leash_z = true;
 AC_PosControl.flags_recalc_leash_xy = true;
 AC_PosControl.flags_reset_desired_vel_to_pos = true;
 AC_PosControl.flags_reset_accel_to_lean_xy = true;
 AC_PosControl.flags_reset_rate_to_accel_z = true;
 AC_PosControl.flags_freeze_ff_z = true;
 AC_PosControl.flags_use_desvel_ff_z = true;
 AC_PosControl.flags_vehicle_horiz_vel_override =false;
 
 AC_PosControl.limit_pos_up = true;
 AC_PosControl.limit_pos_down = true;
 AC_PosControl.limit_vel_up = true;
 AC_PosControl.limit_vel_down = true;
 AC_PosControl.limit_accel_xy = true;
 

 
AC_PosControl.pos_error                 = [0 0 0];
AC_PosControl.pos_target                = [0 0 0];
AC_PosControl.vel_target                = [0 0 0];
AC_PosControl.accel_target              = [0 0 0];
AC_PosControl.accel_target_filter       = [0 0 0];
AC_PosControl.roll_target               = 0;
AC_PosControl.pitch_target              = 0;
AC_PosControl.target_yaw_rate           = 0;
AC_PosControl.vehicle_horiz_vel         = [0 0];
AC_PosControl.vel_desired                      = [0 0 0];
AC_PosControl.vel_error_filter                 = [0 0 0];
AC_PosControl.vel_error                        = [0 0 0];
AC_PosControl.vel_last                         = [0 0 0];
AC_PosControl.accel_desired                    = [0 0 0];
AC_PosControl.vel_z_control_ratio              = 0;

 
 AC_PosControl.pid_vel_xy.flags_reset_filter            = 0;
 AC_PosControl.pid_vel_xy.disable_integrator            = true;
 AC_PosControl.pid_vel_xy.target                        = [0 0];
 AC_PosControl.pid_vel_xy.error                         = [0 0];
 AC_PosControl.pid_vel_xy.error_last                    = [0 0];
 AC_PosControl.pid_vel_xy.integrator                    = [0 0];
 AC_PosControl.pid_vel_xy.derivative                    = [0 0];
%  AC_PosControl.pid_vel_xy.slew_amplitude                = [0 0];
%  AC_PosControl.pid_vel_xy.slew_filterg                  = [0 0];
%  AC_PosControl.pid_vel_xy.last_sample                   = [0 0];
%  AC_PosControl.pid_vel_xy.Dmod                          = [1 1];
%  
 AC_PosControl.pid_accel_z.flags_reset_filter         = 0;
 AC_PosControl.pid_accel_z.disable_integrator         = true;
 AC_PosControl.pid_accel_z.target                     = 0;
 AC_PosControl.pid_accel_z.error                      = 0;
 AC_PosControl.pid_accel_z.error_last                 = 0;
 AC_PosControl.pid_accel_z.integrator                 = 0;
 AC_PosControl.pid_accel_z.derivative                 = 0;
 AC_PosControl.pid_accel_z.slew_amplitude             = 0;
 AC_PosControl.pid_accel_z.slew_filterg               = 0;
 AC_PosControl.pid_accel_z.last_sample                = 0;
 AC_PosControl.pid_accel_z.Dmod                       = 0;

 
 maxClimbRate                           = 5                            ;
 minSinkRate                            = 3                            ;
 timeConstant                           = 3                            ;
 thrDamp                                = 0.7                          ;
 integGain                              = 0.25                         ;
 vertAccLim                             = 3.5                          ;
 spdCompFiltOmega                       = 2                            ;
 rollComp                               = 20                           ;
 spdWeight_inint                        = 0.9                          ;
 ptchDamp                               = 0.7                          ;
 maxSinkRate                            = 7                            ;
 throttle_cruise                        = 40                           ;
 
 
 AP_TECS.timeConstant              = 3;
 AP_TECS.integGain                 = 0.25;
 AP_TECS.rollComp                  = 20 ;
 AP_TECS.ptchDamp                  = 0.7;
 AP_TECS.thrDamp                   = 0.7;
 AP_TECS.throttle_cruise           = 40;
 AP_TECS.throttle_slewrate         = 100;
 AP_TECS.maxClimbRate              = 5;
 AP_TECS.minSinkRate               = 3;
 AP_TECS.maxSinkRate               = 7;
 AP_TECS.vertAccLim                = 3.5;
 AP_TECS.spdCompFiltOmega          = 2;
 AP_TECS.p_ff_throttle             = 0.5;
 AP_TECS.pitch_max                 = 0;
 AP_TECS.pitch_min                 = 0;
%  AP_TECS.pitch_limit_max_cd        = 2000;
%  AP_TECS.pitch_limit_min_cd        = -1500;
 AP_TECS.spdWeight                 = 0.9;
 AP_TECS.throttle_max              = 100;
 AP_TECS.throttle_min              = 0;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 AP_TECS.THRmaxf                   = 1;
 AP_TECS.THRminf                   = 0;
 AP_TECS.throttle_dem              = 0;
 AP_TECS.last_throttle_dem         = 0;
 AP_TECS.integTHR_state            = 0;
 AP_TECS.ff_throttle               = 0;
 
 AP_TECS.hgt_dem                   = 0;
 AP_TECS.hgt_dem_in_old            = 0;
 AP_TECS.hgt_dem_prev              = 0;
 AP_TECS.hgt_dem_adj               = 0;
 AP_TECS.hgt_dem_adj_last          = 0;
 AP_TECS.hgt_rate_dem              = 0;
 
 AP_TECS.pitch_dem                 = 0;
 AP_TECS.pitch_dem_unc             = 0;
 AP_TECS.last_pitch_dem            = 0;
 AP_TECS.PITCHmaxf                 = 0;
 AP_TECS.PITCHminf                 = 0;
 AP_TECS.pitch_max_limit           = 0;
 
 AP_TECS.vel_dot                   = 0;
 AP_TECS.vdot_filter               = [0 0 0 0 0];
 AP_TECS.climb_rate                = 0;
 AP_TECS.EAS_dem                   = 19;
 AP_TECS.TAS_dem                   = 19;
 AP_TECS.TAS_state                 = 19;
 AP_TECS.TAS_dem_adj               = 19;
 AP_TECS.TAS_rate_dem              = 0;
 AP_TECS.integDTAS_state           = 0;
 AP_TECS.TASmax                    = 23;
 AP_TECS.TASmin                    = 15;
 
 AP_TECS.SPE_dem                   = 0;
 AP_TECS.SPE_est                   = 0;
 AP_TECS.SPEdot_dem                = 0;
 AP_TECS.SPEdot                    = 0;
 AP_TECS.SKE_dem                   = 0;
 AP_TECS.SKE_est                   = 0;
 AP_TECS.SKEdot_dem                = 0;
 AP_TECS.SKEdot                    = 0;
 AP_TECS.STE_error                 = 0;
 AP_TECS.STEdot_min                = 0;
 AP_TECS.STEdot_max                = 0;
 AP_TECS.STEdotErrLast             = 0;
 AP_TECS.integSEB_state            = 0;

 AP_L1.L1_xtrack_i_gain            = 0;
 AP_L1.L1_damping                  = 0.75;
 AP_L1.L1_period                   = 15;
 AP_L1.loiter_bank_limit           = 0;
							     
 AP_L1.target_bearing_cd           = 0;
 AP_L1.L1_dist                     = 0;
 AP_L1.crosstrack_error            = 0;
 AP_L1.nav_bearing                 = 0;
 AP_L1.L1_xtrack_i_gain_prev       = 0;
 AP_L1.L1_xtrack_i                 = 0;
 AP_L1.last_Nu                     = 0;
 AP_L1.latAccDem                   = 0;
 AP_L1.WPcircle                    = 0;
 AP_L1.bearing_error               = 0;
 AP_L1.data_is_stale               = 0;
 AP_L1.mode_L1                     = 0;
 AP_L1.reverse                     = 0;
 
Plane.airspeed_max                     =  22;
Plane.airspeed_min                     =  15;
Plane.kff_rudder_mix                   =  1.2;
Plane.scaling_speed                    =  17;
Plane.pitch_limit_max_cd               = 2000;
Plane.pitch_limit_min_cd               = -1500;

Plane.highest_airspeed                 = 0;
Plane.nav_pitch_cd                     = 0;
Plane.kff_throttle_to_pitch            = 0;
Plane.inverted_flight                  = 0;
Plane.smoothed_airspeed                = 0;
Plane.aerodynamic_load_factor          = 0;
Plane.nav_roll_cd                      = 0;
Plane.roll_limit_cd                    = 0;

SRV_Channel.k_rudder                   = 0;
SRV_Channel.k_aileron                  = 0;
SRV_Channel.k_elevator                 = 0;
SRV_Channel.k_throttle                 = 0;
SRV_Channel.k_flap                     = 0;
SRV_Channel.tail_tilt                  = 0;
SRV_Channel.pwm_tail                   = 1000;
SRV_Channel.pwm_out                    = [1000 1000 1000 1000];



 
AC_Attitude.p_angle_roll                                       = 3.3;
AC_Attitude.p_angle_pitch                                      = 3;
AC_Attitude.p_angle_yaw                                        = 3;
AC_Attitude.use_sqrt_controller                                = 1;
AC_Attitude.accel_roll_max                                     = 72000;
AC_Attitude.accel_pitch_max                                    = 30000;
AC_Attitude.accel_yaw_max                                      = 18000;
AC_Attitude.ang_vel_roll_max                                   = 0;
AC_Attitude.ang_vel_pitch_max                                  = 0;
AC_Attitude.ang_vel_yaw_max                                    = 0;
AC_Attitude.input_tc                                           = 0.3;
AC_Attitude.rate_bf_ff_enabled                                 = 1;

AC_Attitude.thrust_error_angle                                 = 0;
AC_Attitude.rate_target_ang_vel                                = [0 0 0];
AC_Attitude.attitude_ang_error                                 = [1 0 0 0];
AC_Attitude.attitude_error_vector                              = [0 0 0];								             								             
AC_Attitude.attitude_target_quat                               = [1 0 0 0];
AC_Attitude.attitude_target_euler_angle                        = [0 0 0];
AC_Attitude.attitude_target_euler_rate                         = [0 0 0];
AC_Attitude.attitude_target_ang_vel                            = [0 0 0];
AC_Attitude.althold_lean_angle_max                             = 0;

%  attitude_target_euler_angle=[roll_target pitch_target yaw_target]*0.01/HD;
%  attitude_target_quat=from_euler(attitude_target_euler_angle);

AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS          = radians(40.0) ;    %minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS          = radians(720.0);   %maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS           = radians(10.0) ;   % minimum body-frame acceleration limit for the stability controller (for yaw axis)
AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS           = radians(120.0) ;   % maximum body-frame acceleration limit for the stability controller (for yaw axis)
AC_Attitude.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX                = 0.8;
AC_Attitude.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN                         = 10 ;
AC_Attitude.AC_ATTITUDE_THRUST_ERROR_ANGLE                     = radians(30.0);               %Thrust angle error above which yaw corrections are limited;


AP_Motors.Kx                                                 = 0                            ;
AP_Motors.thrust_slew_time                                   = 0.3                          ;
AP_Motors.thr_mix_min                                        = 0.1                          ;
AP_Motors.thr_mix_max                                        = 0.5                          ;
																			             
AP_Motors.p_tail_tilt                                        = 3                            ;
AP_Motors.tail_tilt_c2p                                      = -2800                        ;
AP_Motors.current_tilt                                       = 0                            ;
AP_Motors.yaw_headroom                                       = 0                            ;
AP_Motors.thrust_curve_expo                                  = 0.25                         ;
AP_Motors.spin_max                                           = 0.95                         ;
AP_Motors.batt_voltage_max                                   = 0                            ;
AP_Motors.batt_voltage_min                                   = 0                            ;
AP_Motors.batt_current_max                                   = 0                            ;
AP_Motors.pwm_max                                            = 2000                         ;
AP_Motors.pwm_min                                            = 1000                         ;
AP_Motors.spin_min                                           = 0.15                         ;
AP_Motors.spin_arm                                           = 0.1                          ;
AP_Motors.batt_current_time_constant                         = 5                            ;
AP_Motors.throttle_hover                                     = 0.25                         ;
AP_Motors.disarm_disable_pwm                                 = 1                            ;
AP_Motors.spool_up_time                                      = 0.5                          ;
AP_Motors.slew_up_time                                       = 0                            ;%0.5                 
AP_Motors.slew_dn_time                                       = 0                            ;%0.5                 
AP_Motors.safe_time                                          = 1                            ;
AP_Motors.angle_limit_tc                                     = 1                            ;
AP_Motors.angle_boost_enabled                                = 1                            ;
AP_Motors.air_density_ratio                                  = 1                            ;                       
																			             
AP_Motors.batt_current                                       = 0                            ;
AP_Motors.batt_resistance                                    = 0                            ;
AP_Motors.batt_voltage                                       = 0                            ;
AP_Motors.batt_voltage_resting_estimate                      = 0                            ;
AP_Motors.batt_voltage_filt                                  = 0                            ;
AP_Motors.lift_max                                           = 1                            ;
AP_Motors.spool_desired                                      = DesiredSpoolState.SHUT_DOWN  ;
AP_Motors.spool_state                                        = SpoolState.SHUT_DOWN         ;
AP_Motors.angle_boost                                        = 0                            ;
AP_Motors.althold_lean_angle_max                             = 0                            ;

AP_Motors.roll_in                                            = 0                            ;
AP_Motors.pitch_in                                           = 0                            ;
AP_Motors.yaw_in                                             = 0                            ;
AP_Motors.throttle_in                                        = 0                            ;

AP_Motors.roll_factor                                        = [0 0 0 0]                            ;
AP_Motors.pitch_factor                                       = [0 0 0 0]                            ;
AP_Motors.yaw_factor                                         = [0 0 0 0]                            ;

AP_Motors.actuator                                           = [0 0 0 0]                    ;
AP_Motors.thrust_rpyt_out                                    = [0 0 0 0]                    ;

AP_Motors.thrust_boost                                       = 0;
AP_Motors.thrust_boost_ratio                                 = 0;
AP_Motors.thrust_balanced                                    = 0;
AP_Motors.disarm_safe_timer                                  = 0;
AP_Motors.spin_up_ratio                                      = 0;
AP_Motors.throttle_cutoff_frequency                          = 1;
AP_Motors.throttle_filter                                    = 0;
AP_Motors.throttle_limit                                     = 1;
AP_Motors.throttle_avg_max                                   = 0;
AP_Motors.throttle_rpy_mix                                   = 0;
AP_Motors.throttle_rpy_mix_desired                           = 0;
AP_Motors.throttle_thrust_max                                = 1;
AP_Motors.throttle_out                                       = 0;
AP_Motors.thrust_rpyt_out_filt                               = [0 0 0 0];
AP_Motors.land_accel_ef_filter                               = 0;
AP_Motors.motor_lost_index                                   = 0;

AP_Motors.limit_roll                                         = false;
AP_Motors.limit_pitch                                        = false;
AP_Motors.limit_yaw                                          = false;
AP_Motors.limit_throttle_lower                               = false;
AP_Motors.limit_throttle_upper                               = false;

AP_Motors.LAND_CHECK_ANGLE_ERROR_DEG                         = 30;% maximum angle error to be considered landing
AP_Motors.LAND_CHECK_LARGE_ANGLE_CD                          = 15;% maximum angle target to be considered landing
AP_Motors.LAND_CHECK_ACCEL_MOVING                            = 3.0;% maximum acceleration after subtracting gravity
AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CW                     = -1;
AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CCW                    = 1;
AP_Motors.AC_ATTITUDE_CONTROL_MAX                            = 5;
AP_Motors.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX       = 0.8;

AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN =10;
AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX=0.8;
althold_lean_angle_max=0;
attitude_target_quat=[1 0 0 0];
rate_target_ang_vel=[0 0 0];

Copter_Plane.aspeed_cp                              = 30         ;          
Copter_Plane.aspeed_c2p                             = 8          ;          
Copter_Plane.aspeed_c2ps                            = 18         ;          
Copter_Plane.p_plane_c2p                            = 0.6        ;          
Copter_Plane.p_plane_cp                             = 0.4        ;          
Copter_Plane.p_k_elevator_c2p                       = 2          ;          
Copter_Plane.pitch_target_c2p                       = 0          ;          
Copter_Plane.tail_tilt_c2p                          = -2800      ;          
Copter_Plane.k_throttle_c2p                         = 0.4        ;          
Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p           = 0.2        ;          
Copter_Plane.thr_out_min_c2p                        = 0.5        ;          
Copter_Plane.yaw_max_c2p                            = 0.1        ;          
Copter_Plane.p_tilt_pitch_target                    = 0.4        ;          
Copter_Plane.tail_tilt_p2c                          = -1300      ;          
Copter_Plane.p_plane_p2c                            = 1          ;          
Copter_Plane.pitch_target_p2c                       = 500        ;          
Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c    = 0.05       ;          
Copter_Plane.Fix2Rotor_delay_s                      = 0.8        ;          
Copter_Plane.tail_tilt_rate                         = 1500       ;          



Copter_Plane.inint                                 = 1            ;                                                                                              
Copter_Plane.mode                                  = 0            ; 
Copter_Plane.EAS_dem_cm                            = 2000         ; 
Copter_Plane.hgt_dem_cm                            = 0            ; 
Copter_Plane.roll_target_pilot                     = 0            ; 
Copter_Plane.pitch_target_pilot                    = 0            ; 
Copter_Plane.heading_hold                          = 0            ;                                                                                               
Copter_Plane.climb_rate_cms                        = 0            ; 


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
		
Copter_Plane.prev_WP                               = [40,100]*1e7                           ; 
Copter_Plane.next_WP                               = [40,100.01]*1e7                        ; 
Copter_Plane.dist_min                              = 50                                     ; 

Copter_Plane.center_WP                             = [40,100]*1e7                           ; 
Copter_Plane.loc_origin                            = [40,100]*1e7                           ; 
Copter_Plane.current_loc                           = [40,100]*1e7                           ; 
Copter_Plane.loc                                   = loc                                    ; 
Copter_Plane.L1_radius                             = 60                                     ;
Copter_Plane.radius                                = 80                                     ;
Copter_Plane.loiter_direction                      = 1                                      ;     

Copter_Plane.k_flap_TakeOff                        = -3500                                  ; 
Copter_Plane.k_flap_Land                           = 3500                                   ; 
Copter_Plane.thr_out_min                           = 0                                      ; 
Copter_Plane.throttle_ground                       = 0.45                                   ; 
Copter_Plane.throttle_off_rate                     = 0.03                                   ; 

Copter_Plane.take_off_land                         = 0                                      ; 
Copter_Plane.inint_hgt                             = 0                                      ; 

Copter_Plane.weathervane_gain                      = 1.2                                    ;
Copter_Plane.weathervane_min_roll                  = 4                                      ;           
Copter_Plane.weathervane_last_output               = 0                                      ;
Copter_Plane.yaw_rate_max                          = 50                                     ;

Copter_Plane.vel_forward_gain                       = 1          ;
Copter_Plane.vel_forward_min_pitch                  = -4         ;
Copter_Plane.vel_forward_tail_tilt_max              = 2000       ;
Copter_Plane.vel_forward_integrator                 = 0          ;
Copter_Plane.arspeed_filt                           = 5          ;
Copter_Plane.arspeed_temp                           = 0          ;
Copter_Plane.disable_AP_roll_integrator             = 0          ;
Copter_Plane.disable_AP_pitch_integrator            = 0          ;
Copter_Plane.disable_AP_yaw_integrator              = 0          ;
Copter_Plane.disable_AP_rate_roll_gains_D           = 0;
Copter_Plane.disable_AP_rate_pitch_roll_ff          = 0;
Copter_Plane.disable_AP_rate_pitch_gains_D          = 0;
Copter_Plane.disable_AP_rate_yaw_K_FF               = 0;


LOCATION_SCALING_FACTOR                = 0.011131884502145034;
LOCATION_SCALING_FACTOR_INV            = 89.83204953368922;   
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
climb_rate_cms=0;
slew_yaw=6000;
angle_boost_enabled=1;
angle_limit_tc=1;

    k_flap=0;
    thr_out_min=0;
    take_off_land=0;                        
    ff_throttle=0;
    GRAVITY_MSS=9.80665;
    disable_integrator_pitch=0;
    disable_integrator_roll=0;
    disable_integrator_yaw=0;
    ATC_RAT_PIT_I   = 0.0;
    ATC_RAT_RLL_I    =0.0;
    ATC_RAT_YAW_I    =0.0;
    K_FF_yaw=0; 
    roll_ff_pitch=0;
    POSCONTROL_ACC_Z_I=0.0;    % vertical acceleration controller I gain default
    POSCONTROL_ACC_Z_FILT_HZ=0;    % vertical acceleration controller input filter default
    POSCONTROL_VEL_XY_I=0; % horizontal velocity controller I gain default

    spdWeight=0;

    POSCONTROL_THROTTLE_CUTOFF_FREQ=0;
    gains_D_pitch=0;
    gains_D_roll=0;
    tail_tilt=0;
    inint_hgt=1;   
    pitch_max=0;
    pitch_min=0;
    hgt_dem_cm=0;
    arspeed_temp=0;
    loiter_bank_limit=0;
    weathervane_last_output=0;
    
    nav_pitch_cd=0;
    nav_roll_cd=0;
    inverted_flight=0;
    k_rudder=0;
    k_aileron=0;
    k_aileronL=0;
    k_aileronR=0;
    k_throttle=0.0;
    k_elevator=0;
    desired_rate_pitch=0;
    desired_rate_roll=0;
    
    kff_throttle_to_pitch=0;
    smoothed_airspeed=0;
    aerodynamic_load_factor=1;
    vdot_filter=zeros(5,1);
    rot_body_to_ned=eye(3,3);
    imu_filt=10;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 vel_forward_last_pct=0;
 vel_forward_tail_tilt_max=2000;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lux=310/1000;
Luy=210/1000;
Ldx=205/1000;
Ldy=355.5/1000;
Lu=hypot(Lux,Luy);
Ld=hypot(Ldx,Ldy);
Ku=32;
% Kd=18;
Kd=Ku*Luy/Ldy;
Kx=(Ku-Kd)/(Ku+Kd);
Qd=10;
Qu=asind(Kd*sind(Qd)*Ld/(Ku*Lu));
Kc=(Ku*Lux)/(Ldx*Kd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%sl
    current_loc=[40,100]*1e7;
    loc_origin=[40,100]*1e7;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    center_WP=[40,100]*1e7;

PathModeOut_sl.headingCmd=10000;
PathModeOut_sl.groundspeedCmd=0;
PathModeOut_sl.heightCmd=10000;
PathModeOut_sl.flightTaskMode=ENUM_FlightTaskMode.GroundStandByMode;
PathModeOut_sl.maxClimbSpeed=300;
PathModeOut_sl.turnCenterLL=[40,100]*1e7;
PathModeOut_sl.prePathPoint_LLA=[0 0 0];
PathModeOut_sl.curPathPoint_LLA=[0 0 0];

algo_remote_ct_st.isRemoteConnected=1;
algo_remote_ct_st.mode=0;
algo_remote_ct_st.roll=0;
algo_remote_ct_st.pitch=0;
algo_remote_ct_st.yaw=0;
algo_remote_ct_st.throttle=0; 
algo_remote_ct_st.tail_anglein=0;
algo_remote_ct_st.tail_throttle_pwm=0;


algo_dbg_param.headingCmd=0;
algo_dbg_param.groundspeedCmd=0;
algo_dbg_param.heightCmd=0;
algo_dbg_param.flightTaskMode=ENUM_FlightTaskMode.Rotor2Fix_Mode;
algo_dbg_param.maxClimbSpeed=0; 

 Curr_sate.accel_x=0;
 Curr_sate.accel_y=0;
 Curr_sate.accel_z=0;
 Curr_sate.gyro_x=0;
 Curr_sate.gyro_y=0;
 Curr_sate.gyro_z=0;
 Curr_sate.EAS_Algo=0;
 Curr_sate.EAS2TAS_Algo=1;
 curVelNED=[0 0 0];
 Curr_sate.curLLA=[0 0 0];
 Curr_sate.pitchd=0;
 Curr_sate.rolld=0;
 Curr_sate.yawd=0;
 Curr_sate.NAV_alt=0;
 
 Sevro_pwm.k_aileron=0;
 Sevro_pwm.k_elevator=0;
 Svero_pwm.k_rudder=0;
 Svero_pwm.tail_tilt=0;
 Svero_pwm.pwm_out=[1000 1000 1000 1000];
 Svero_pwm.pwm_tail=1000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mode=0;
    mode_state=0;
    ts=0;
    dt=0.005;

    pwm_out=[1000 1000 1000 1000];
    highest_airspeed=30;
    scaling_speed=17;

    TASmax=23;
    TASmin=15;
    THRmaxf=1;
    THRminf=0;
    throttle_min=0;
    throttle_max=100; 
    pitch_limit_min_cd=-1500;
    pitch_limit_max_cd=2000;
    pitch_max_limit=90;
    roll_limit_cd=2500;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% wind %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    HD=180/pi;
    pitch=10/HD;
    roll=00/HD;
    yaw=0/HD;
    
     curr_vel=[0 0 0];
     curr_pos=[0 0 ];

    gyro_x=000/HD;
    gyro_y=000/HD;
    gyro_z=0;

    accel_x=0;
    accel_y=0;    
    accel_z=0;
    VN=0;
    VE=0;  
    Vz=0;
    EAS=17;
    aspeed=17;
    EAS2TAS=1;   
    groundspeed_vector=[0.1 0.1];
    L1_radius=60;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    prev_WP=[40,100]*1e7;
    next_WP=[40,100.01]*1e7;
    dist_min=50;
%     loc=[0,      40*1e7,     100*1e7;
%          1,      40.01*1e7,  100.01*1e7;
%          2,      40.02*1e7,  100.01*1e7;
%          3,      40.02*1e7,  100.02*1e7;
%          4,      40.01*1e7,  100.02*1e7;
%          5,      40.01*1e7,  100.01*1e7;
%          99,     0*1e7,      0*1e7;    ];
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
    
    mode_L1=0;
    roll_target_pilot=0;
    pitch_target_pilot=0; 
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AC_AttitudeControl matlab sim  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ATC_ACCEL_P_MAX = 36397.976562;
ATC_ACCEL_R_MAX = 33111.574219;
ATC_ACCEL_Y_MAX = 36397.976562;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    POSCONTROL_ACCELERATION_MIN=50.0;% minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
%     POSCONTROL_ACCEL_XY=        100.0;%default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
    POSCONTROL_ACCEL_XY_MAX=    458;%max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
    POSCONTROL_STOPPING_DIST_UP_MAX=         300.0;%max stopping distance (in cm) vertically while climbing
    POSCONTROL_STOPPING_DIST_DOWN_MAX=       200.0;%max stopping distance (in cm) vertically while descending

%     POSCONTROL_SPEED=           300.0;%default horizontal speed in cm/s
    POSCONTROL_SPEED_DOWN=     -300.0;%default descent rate in cm/s
    POSCONTROL_SPEED_UP=        300.0;%default climb rate in cm/s

%     POSCONTROL_ACCEL_Z=         250.0;%default vertical acceleration in cm/s/s.

    POSCONTROL_LEASH_LENGTH_MIN=100.0;%minimum leash lengths in cm

    POSCONTROL_DT_50HZ=         0.02;% time difference in seconds for 50hz update rate
    POSCONTROL_DT_400HZ=        0.0025;% time difference in seconds for 400hz update rate

    POSCONTROL_ACTIVE_TIMEOUT_US=200000 ;% position controller is considered active if it has been called within the past 0.2 seconds

    % low-pass filter on velocity error (unit: hz)
    % low-pass filter on acceleration (unit: hz)
    POSCONTROL_JERK_RATIO=      1.0;% Defines the time it takes to reach the requested acceleration

    POSCONTROL_OVERSPEED_GAIN_Z=2.0;% gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range
    
    accel_xy_filt_hz=POSCONTROL_ACCEL_FILTER_HZ;

 recalc_leash_z=1;
 limit_pos_down=0;
 limit_pos_up=0;
 pos_error=[0 0 0];
 pos_target=[0 0 0];
 vel_target=[0 0 0];
 limit_vel_up=0;
 limit_vel_down=0;
 use_desvel_ff_z=1;
 reset_rate_to_accel_z=1;
 curr_alt=0;
 vel_desired=[0 0 0];
 vel_last=[0 0 0];
 accel_desired=[0 0 0];
 freeze_ff_z=0;
 vel_error=[0 0 0];
 vel_error_input=0;
 accel_target=[0 0 0];
 reset_accel_to_throttle=1;
 accel_error=[0 0 0];
 z_accel_meas=0;
 pid_accel_z_reset_filter=1;
 pid_accel_z_input=0;
 pid_accel_z_derivative=0;
 pid_accel_z_integrator=0;
 throttle_lower=0;
 throttle_upper=0;
 throttle_input=0;
 %%%%%%%%%%%%%%%%%%
 roll_target=0;
 pitch_target=0;
 yaw_target=0;
 target_yaw_rate=0;
 



 
 %%%%%%%%%%%%%%%%%%%%%%
 throttle_rpy_mix=0;

 pid_vel_xy_reset_filter=1;
 pid_vel_xy_input=[0 0];
 pid_vel_xy_derivative=[0 0];
 pid_vel_xy_integrator=[0 0];
 limit_accel_xy=0;
 motors_limit_throttle_upper=0;
 reset_accel_to_lean_xy=1;
 accel_xy_input=[0 0];
 accel_xy_angle_max=2000;%%dai cha

 recalc_leash_xy=1;
 reset_desired_vel_to_pos=1;
 accel_last_z_cms=0;
 is_active_z=1;
 is_active_xy=1;

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 inint=1;


roll_factor=[0 0 0 0];
pitch_factor=[0 0 0 0];
yaw_factor=[0 0 0 0];

limit_roll_pitch=0;
limit_yaw=0;
air_density_ratio=1;
thrust_boost=0;

angle_boost=0;
throttle_avg_max=1;
throttle_cutoff_frequency=1;
throttle_thrust_max=1;
throttle_rpy_mix_desired=0;
AC_ATTITUDE_CONTROL_MAX=5;
roll_in=0;
pitch_in=0;
yaw_in=0;
armed=1;
% throttle_hover=0.4816;
% throttle_in=0.4816;
% throttle_filter=throttle_in;
% pwm_out=ones(1,4)*(throttle_hover*1000+1000);

throttle_in=0.4816;
throttle_filter=0;
thrust_boost_ratio=0;
thrust_rpyt_out=[0 0 0 0];
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AP_PitchController

    gains_rmax_pitch=0;
    max_rate_neg=0;

    last_out_pitch=0;
    pid_info_I_pitch=0;
    pid_info_P_pitch=0;
    pid_info_FF_pitch=0;
    pid_info_D_pitch=0;
    pid_info_desired_pitch=0;
    pid_info_actual_pitch=0;
    gains_FF_pitch=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_RollController

    last_out_roll=0;
    pid_info_I_roll=0;
    pid_info_P_roll=0;
    pid_info_FF_roll=0;
    pid_info_D_roll=0;
    pid_info_desired_roll=0;
    pid_info_actual_roll=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%  AP_YawController 
	% @Param: SLIP
	% @DisplayName: Sideslip control gain
	% @Description: Gain from lateral acceleration to demanded yaw rate for aircraft with enough fuselage area to detect lateral acceleration and sideslips. Do not enable for flying wings and gliders. Actively coordinates flight more than just yaw damping. Set after YAW2SRV_DAMP and YAW2SRV_INT are tuned.
	% @Range: 0 4
	% @Increment: 0.25
    % @User: Advanced
% 	AP_GROUPINFO("SLIP",    0, AP_YawController, _K_A,    0),

    
    K_D_last_yaw=0;
    integrator_yaw=0;
    last_out_yaw=0;
    pid_info_I_yaw=0;
    pid_info_D_yaw=0;
    last_rate_hp_out_yaw=0;
    last_rate_hp_in_yaw=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_TECS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SKE_dem=0;
    SPE_dem=0;
    SPE_est=0;
    SKE_est=0;
    STE_error=0;
    SPEdot_dem=0;
    SKEdot_dem=0;
    STEdot_min=0;
    STEdot_max=0;
    SPEdot=0;
    SKEdot=0;
    STEdotErrLast=0;
    throttle_dem=0;
    last_throttle_dem=0;
    integTHR_state=0;
    hgt_dem=0;
    PITCHmaxf=0;
    PITCHminf=0;
    TAS_rate_dem=0;
    hgt_dem_in_old=0;
    max_sink_rate=0;
    hgt_dem_prev=0;
    hgt_dem_adj=0;
    hgt_dem_adj_last=0;
    hgt_rate_dem=0;
    height=0;
    climb_rate=0;
    pitch_dem=0;
    integSEB_state=0;
    pitch_dem_unc=0;
    last_pitch_dem=0;

    EAS_dem=17;
    TAS_dem=17;
    TAS_dem_adj=TAS_dem;
    TAS_state=17;
    integDTAS_state=0;
    vel_dot=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_L1_Control

 LOCATION_SCALING_FACTOR = 0.011131884502145034;
 LOCATION_SCALING_FACTOR_INV = 89.83204953368922;
    target_bearing_cd=0;
    L1_dist=0;
    crosstrack_error=0;
    nav_bearing=0;
    L1_xtrack_i_gain_prev=0;
    L1_xtrack_i=0;
    last_Nu=0;
    latAccDem=0;
    WPcircle=0;
    bearing_error=0;
    data_is_stale=0;
    reverse=0;
%Bank angle command based on angle between aircraft velocity vector and reference vector to path.
%S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
%Proceedings of the AIAA Guidance, Navigation and Control
%Conference, Aug 2004. AIAA-2004-4900.
%Modified to use PD control for circle tracking to enable loiter radius less than L1 length
%Modified to enable period and damping of guidance loop to be set explicitly
%Modified to provide explicit control over capture angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% // parameters for the motor class
% const AP_Param::GroupInfo AP_MotorsMulticopter::var_info[] = {
%     // 0 was used by TB_RATIO
%     // 1,2,3 were used by throttle curve
%     // 5 was SPIN_ARMED
% 
%     // @Param: YAW_HEADROOM
%     // @DisplayName: Matrix Yaw Min
%     // @Description: Yaw control is given at least this pwm in microseconds range
%     // @Range: 0 500
%     // @Units: PWM
%     // @User: Advanced
%     AP_GROUPINFO("YAW_HEADROOM", 6, AP_MotorsMulticopter, _yaw_headroom, AP_MOTORS_YAW_HEADROOM_DEFAULT),
% 
%     // 7 was THR_LOW_CMP
% 

