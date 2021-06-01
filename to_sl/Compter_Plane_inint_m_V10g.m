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
ts                                                       =0;
dt                                                       = 0.012;
HD                                                       = 180/pi;
GRAVITY_MSS                                              = 9.80665;
LOCATION_SCALING_FACTOR                                  = 0.011131884502145034;% 1e-7*地球半径/HD
LOCATION_SCALING_FACTOR_INV                              = 89.83204953368922;

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
loc.num=(1:20)';
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_V10_norm();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
AC_PosControl.pid_vel_xy_p_wind                          = 1;                            
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
Copter_Plane.State                                       =  ENUM_State.E_Copter;
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
Copter_Plane.disable_forward_throttle                     = false                                   ; 


AP_Motors.batt_current                                   = 0                            ;
AP_Motors.batt_resistance                                = 0                            ;
AP_Motors.batt_voltage                                   = 0                            ;
AP_Motors.batt_voltage_resting_estimate                  = 0                            ;
AP_Motors.batt_voltage_filt                              = 0                            ;
AP_Motors.lift_max                                       = 1                            ;
AP_Motors.spool_desired                                  = SpoolState.GROUND_IDLE  ;
AP_Motors.spool_state                                    = SpoolState.GROUND_IDLE         ;
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

