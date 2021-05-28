function param_V10_init()
%控制参数 V10
global AC_rate_roll_pid
global AC_rate_pitch_pid
global AC_rate_yaw_pid
global AC_Attitude
global AC_PosControl
global AP_rate_roll
global AP_rate_pitch
global AP_rate_yaw
global AP_TECS
global AP_L1
global Plane
global Copter_Plane
global AP_Motors
global Test_w

        AC_rate_roll_pid.kp                                      = 0.28    ;%ATC_RAT_RLL_P
        AC_rate_roll_pid.ki                                      = 0.1    ;%ATC_RAT_RLL_I
        AC_rate_roll_pid.kd                                      = 0      ;%ATC_RAT_RLL_D
        AC_rate_roll_pid.kff                                     = 0      ;%ATC_RAT_RLL_FF
        AC_rate_roll_pid.kimax                                   = 0.25   ;%ATC_RAT_RLL_IMAX
        AC_rate_roll_pid.filt_T_hz                               = 20     ;%ATC_RAT_RLL_FLTT
        AC_rate_roll_pid.filt_E_hz                               = 20     ;%ATC_RAT_RLL_FLTE
        AC_rate_roll_pid.filt_D_hz                               = 20     ;%ATC_RAT_RLL_FLTD
        AC_rate_roll_pid.slew_rate_max                           = 0      ;%ATC_RAT_RLL_SMAX
        AC_rate_pitch_pid.kp                                     = 0.20   ;%ATC_RAT_PIT_P
        AC_rate_pitch_pid.ki                                     = 0.1    ;%ATC_RAT_PIT_I
        AC_rate_pitch_pid.kd                                     = 0      ;%ATC_RAT_PIT_D
        AC_rate_pitch_pid.kff                                    = 0      ;%ATC_RAT_PIT_FF
        AC_rate_pitch_pid.kimax                                  = 0.25   ;%ATC_RAT_PIT_IMAX
        AC_rate_pitch_pid.filt_T_hz                              = 20     ;%ATC_RAT_PIT_FLTT
        AC_rate_pitch_pid.filt_E_hz                              = 20     ;%ATC_RAT_PIT_FLTE
        AC_rate_pitch_pid.filt_D_hz                              = 20     ;%ATC_RAT_PIT_FLTD
        AC_rate_pitch_pid.slew_rate_max                          = 0      ;%ATC_RAT_PIT_SMAX
        AC_rate_yaw_pid.kp                                       = 0.19   ;%ATC_RAT_YAW_P
        AC_rate_yaw_pid.ki                                       = 0.0    ;%ATC_RAT_YAW_I
        AC_rate_yaw_pid.kd                                       = 0      ;%ATC_RAT_YAW_D
        AC_rate_yaw_pid.kff                                      = 0      ;%ATC_RAT_YAW_FF
        AC_rate_yaw_pid.kimax                                    = 0.0    ;%ATC_RAT_YAW_IMAX
        AC_rate_yaw_pid.filt_T_hz                                = 20     ;%ATC_RAT_YAW_FLTT
        AC_rate_yaw_pid.filt_E_hz                                = 20     ;%ATC_RAT_YAW_FLTE
        AC_rate_yaw_pid.filt_D_hz                                = 5      ;%ATC_RAT_YAW_FLTD
        AC_rate_yaw_pid.slew_rate_max                            = 0      ;%ATC_RAT_YAW_SMAX
        AC_Attitude.p_angle_roll                                 = 3.3    ;%ATC_ANG_RLL_P
        AC_Attitude.p_angle_pitch                                = 3      ;%ATC_ANG_PIT_P
        AC_Attitude.p_angle_yaw                                  = 3      ;%ATC_ANG_YAW_P
        AC_Attitude.input_tc                                     = 0.3    ;%ATC_ANG_LIM_TC
        AC_PosControl.p_pos_z                                    = 1      ;%PSC_POSZ_P
        AC_PosControl.p_vel_z                                    = 1      ;%PSC_VELZ_P
        AC_PosControl.pid_accel_z.kp                             = 0.9    ;%PSC_ACCZ_P
        AC_PosControl.pid_accel_z.ki                             = 0.25   ;%PSC_ACCZ_I
        AC_PosControl.pid_accel_z.kd                             = 0.3    ;%PSC_ACCZ_D
        AC_PosControl.pid_accel_z.kff                            = 0      ;%PSC_ACCZ_FF
        AC_PosControl.pid_accel_z.kimax                          = 200    ;%PSC_ACCZ_IMAX
        AC_PosControl.pid_accel_z.filt_T_hz                      = 20     ;%PSC_ACCZ_FLTT
        AC_PosControl.pid_accel_z.filt_E_hz                      = 4      ;%PSC_ACCZ_FLTE
        AC_PosControl.pid_accel_z.filt_D_hz                      = 20     ;%PSC_ACCZ_FLTD
        AC_PosControl.pid_accel_z.slew_rate_max                  = 0      ;%PSC_ACCZ_SMAX
        AC_PosControl.p_pos_xy                                   = 1.1    ;%PSC_POSXY_P
        AC_PosControl.pid_vel_xy.kp                              = 1.1    ;%PSC_VELXY_P
        AC_PosControl.pid_vel_xy.ki                              = 0.15   ;%PSC_VELXY_I
        AC_PosControl.pid_vel_xy.kd                              = 0.12   ;%PSC_VELXY_D
        AC_PosControl.pid_vel_xy.kff                             = 0      ;%PSC_VELXY_FF
        AC_PosControl.pid_vel_xy.kimax                           = 1000   ;%PSC_VELXY_IMAX
        AC_PosControl.pid_vel_xy.filt_T_hz                       = 20     ;%PSC_VELXY_FLTT
        AC_PosControl.pid_vel_xy.filt_E_hz                       = 2      ;%PSC_VELXY_FLTE
        AC_PosControl.pid_vel_xy.filt_D_hz                       = 5      ;%PSC_VELXY_FLTD
        AC_PosControl.vibe_comp_enabled                          = true   ;%VIBE_COMP
        AC_PosControl.speed_down_cms                             = -300   ;%PSC_SPEED_DOWN
        AC_PosControl.speed_up_cms                               = 300    ;%PSC_SPEED_UP
        AC_PosControl.speed_cms                                  = 300    ;%PSC_SPEED
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
        AP_rate_pitch.gains_tau                                  = 1      ;%PTCH2SRV_TCONST
        AP_rate_pitch.gains_rmax                                 = 0      ;%PTCH2SRV_RMAX_UP
        AP_rate_pitch.max_rate_neg                               = 0      ;%PTCH2SRV_RMAX_DN
        AP_rate_pitch.gains_P                                    = 1.2    ;%PTCH2SRV_P
        AP_rate_pitch.gains_D                                    = 0.35   ;%PTCH2SRV_D
        AP_rate_pitch.gains_I                                    = 0.02   ;%PTCH2SRV_I
        AP_rate_pitch.gains_FF                                   = 0      ;%PTCH2SRV_FF
        AP_rate_pitch.gains_imax                                 = 3000   ;%PTCH2SRV_IMAX
        AP_rate_pitch.roll_ff                                    = 0.85   ;%PTCH2SRV_RLL
        AP_rate_pitch.slew_rate_max                              = 0      ;%PTCH2SRV_SMAX
        AP_rate_yaw.K_A                                          = 0.25   ;%YAW2SRV_SLIP
        AP_rate_yaw.K_I                                          = 0.05   ;%YAW2SRV_INT
        AP_rate_yaw.K_D                                          = 0.25   ;%YAW2SRV_DAMP
        AP_rate_yaw.K_FF                                         = 1      ;%YAW2SRV_RLL
        AP_rate_yaw.imax                                         = 1500   ;%YAW2SRV_IMAX
        AP_TECS.maxClimbRate                                     = 5      ;%TECS_CLMB_MAX
        AP_TECS.minSinkRate                                      = 3      ;%TECS_SINK_MIN
        AP_TECS.timeConstant                                     = 3      ;%TECS_TIME_CONST
        AP_TECS.integGain                                        = 0.25   ;%TECS_INTEG_GAIN
        AP_TECS.rollComp                                         = 25     ;%TECS_RLL2THR
        AP_TECS.ptchDamp                                         = 0.7    ;%TECS_PTCH_DAMP
        AP_TECS.thrDamp                                          = 0.7    ;%TECS_THR_DAMP
        AP_TECS.throttle_cruise                                  = 40     ;%TRIM_THROTTLE
        AP_TECS.maxSinkRate                                      = 7      ;%TECS_SINK_MAX
        AP_TECS.vertAccLim                                       = 3.5    ;%TECS_VERT_ACC
        AP_TECS.p_ff_throttle                                    = 0.5    ;%TECS_P_FF_THR
        AP_L1.L1_damping                                         = 0.75   ;%NAVL1_DAMPING
        AP_L1.L1_period                                          = 17     ;%NAVL1_PERIOD
        AP_L1.L1_xtrack_i_gain                                   = 0.1    ;%NAVL1_XTRACK_I
        Plane.airspeed_max                                       =  26    ;%ARSPD_MAX
        Plane.airspeed_min                                       =  17    ;%ARSPD_MIN
        Plane.kff_rudder_mix                                     =  1.2   ;%KFF_RDDRMIX
        Plane.scaling_speed                                      =  17    ;%SCALING_SPEED
        Plane.pitch_limit_max_cd                                 = 1500   ;%LIM_PITCH_MAX
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
        Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ                    = 4      ;%PSC_ACCZ_FIL
        Copter_Plane.thr_out_min_c2p                             = 0.5    ;%THR_OUT_MIN_C2P
        Copter_Plane.yaw_max_c2p                                 = 0.1    ;%YAW_MAX_C2P
        Copter_Plane.p_tilt_pitch_target                         = 0.4    ;%P_TILT_PIT_TAR
        Copter_Plane.tail_tilt_p2c                               = -1300  ;%TAIL_TILT_P2C
        Copter_Plane.p_plane_p2c                                 = 1      ;%P_PLANE_P2C
        Copter_Plane.pitch_target_p2c                            = 500    ;%PITCH_TAR_P2C
        Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c         = 0.05   ;%PSC_ACCZ_FLTP2C
        Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ             = 3      ;%PSC_ACCZ_FLT
        Copter_Plane.Fix2Rotor_delay_s                           = 0.8    ;%P2C_DELAY_S
        Copter_Plane.tail_tilt_rate                              = 1500   ;%TAIL_TILT_RATE
        Copter_Plane.heading_hold                                = 0      ;%HEADING_HOLD
        Copter_Plane.spdWeight                                   = 0.9    ;%SPDWEINGHT
        Copter_Plane.roll_limit_cd                               = 3800   ;%RLL_LIMIT_CD
        Copter_Plane.EAS_dem_cm                                  = 2000   ;%EAS_DEM_CM
        Copter_Plane.radius                                      = 80     ;%RADIUS
        Copter_Plane.loiter_direction                            = 1      ;%LOITER_DIR
        Copter_Plane.k_flap_TakeOff                              = -3500  ;%K_FLAP_TAKEOFF
        Copter_Plane.k_flap_Land                                 = 4300   ;%K_FLAP_LAND
        Copter_Plane.thr_out_min                                 = 0.38    ;%THR_OUT_MIN
        Copter_Plane.throttle_ground                             = 0.41   ;%THR_GROUND
        Copter_Plane.throttle_off_rate                           = 0.03   ;%THR_OFF_RATE
        Copter_Plane.weathervane_gain                            = 0.7    ;%WVANE_GAIN
        Copter_Plane.weathervane_min_roll                        = 4      ;%WVANE_MINROLL
        Copter_Plane.yaw_rate_max                                = 100    ;%YAW_RATE_MAX
        Copter_Plane.vel_forward_gain                            = 1      ;%VFWD_GAIN
        Copter_Plane.vel_forward_min_pitch                       = -4     ;%VFWD_MIN_PIT
        Copter_Plane.vel_forward_tail_tilt_max                   = 2000   ;%VFWD_MAX_TAIL
        Copter_Plane.arspeed_filt                                = 5      ;%ARSPD_FILT
        AP_Motors.current_tilt                                   = 0      ;%CURR_TILT
        AP_Motors.throttle_hover                                 = 0.5    ;%THST_HOVER
        Test_w.start                                             = 0      ;%TEST_W_START
        Test_w.ws                                                = 0.2    ;%TEST_W_WS
        Test_w.we                                                = 1      ;%TEST_W_WE
        Test_w.dw                                                = 0.2    ;%TEST_W_DW
        Test_w.n                                                 = 5      ;%TEST_W_N
        Test_w.Amp                                               = 0.15   ;%TEST_W_AMP
        Test_w.offset                                            = 0.0    ;%TEST_W_OFFEST
        Test_w.Mode                                              = ENUM_Test_mode.step_w;%TEST_W_MODE
        Test_w.channel                                           = ENUM_Test_channel.throttle_in_t;%TEST_W_CHANNEL
        
        AC_rate_roll_pid.slew_rate_tau                           = 0.1    ;%ATC_RAT_RLL_STAU
        AC_rate_pitch_pid.slew_rate_tau                          = 0.1    ;%ATC_RAT_PIT_STAU
        AC_rate_yaw_pid.slew_rate_tau                            = 0.1    ;%ATC_RAT_YAW_STAU
        AC_PosControl.pid_accel_z.slew_rate_tau                  = 0.1    ;%PSC_ACCZ_STAU
        AP_rate_roll.slew_rate_tau                               = 0.1    ;%RLL2SRV_TAU
        AP_rate_pitch.slew_rate_tau                              = 0.1    ;%PTCH2SRV_TAU
        AP_TECS.spdCompFiltOmega                                 = 2      ;%TECS_SPD_OMEGA
        AP_TECS.pitch_max                                        = 0      ;%TECS_PITCH_MAX
        AP_TECS.pitch_min                                        = 0      ;%TECS_PITCH_MIN
        AP_TECS.throttle_max                                     = 100    ;%TECS_THR_MAX
        AP_TECS.throttle_min                                     = 0      ;%TECS_THR_MIN
        AP_TECS.throttle_slewrate                                = 100    ;%THR_SLEWRATE
        AP_TECS.spdWeight                                        = 0.9    ;%TECS_SPDWEIGHT
        Copter_Plane.dist_min                                    = 50     ;%DIST_MIN
        Copter_Plane.L1_radius                                   = 60     ;%L1_RADIUS
        AP_L1.loiter_bank_limit                                  = 0      ;%NAVL1_LIM_BANK
        
        AP_Motors.Kx                                             = 0                            ;
        AP_Motors.thrust_slew_time                               = 0.3                          ;
        AP_Motors.thr_mix_min                                    = 0.1                          ;
        AP_Motors.thr_mix_max                                    = 0.5                          ;
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
        
        AC_PosControl.accel_z_cms                                = 100;%POSCONTROL_ACCEL_Z
        AC_PosControl.accel_cms                                  = 100;%POSCONTROL_ACCEL_XY
        AC_PosControl.leash                                      = 100;%POSCONTROL_LEASH_LENGTH_MIN
        AC_PosControl.leash_down_z                               = 100;%POSCONTROL_LEASH_LENGTH_MIN
        AC_PosControl.leash_up_z                                 = 100;%POSCONTROL_LEASH_LENGTH_MIN
        AC_PosControl.accel_xy_filt_hz                           = 10;%POSCONTROL_ACCEL_FILTER_HZ
        AC_PosControl.accel_last_z_cms                           = 0;%
        
        AC_PosControl.POSCONTROL_VEL_ERROR_CUTOFF_FREQ           = 1.5;
        AC_PosControl.POSCONTROL_VIBE_COMP_I_GAIN                = 0.125;
        AC_PosControl.POSCONTROL_VIBE_COMP_P_GAIN                = 0.25;
        AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ            = 10;
        AC_PosControl.POSCONTROL_ACCEL_XY_MAX                    = 257;
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
end