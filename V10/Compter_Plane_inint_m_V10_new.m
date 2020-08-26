% %mode_Multicopter
 load bus_comper.mat
% load IOBusInfo_V1000_20200807.mat
pianzhuanjiao

m_kg_V1000=5;
Jx=186222*1e-6;
Jy=164400*1e-6;
Jz=336920*1e-6;
% Jx=186222*1e-5;
% Jy=164400*1e-5;
% Jz=336920*1e-5;
J_V1000=diag([Jx Jy Jz]);

m_kg_V10=26;
Jx_v10=4.17029;
Jy_v10=8.07546;
Jz_v10=4.01077;
% Jx=186222*1e-5;
% Jy=164400*1e-5;
% Jz=336920*1e-5;
J_V10=diag([Jx_v10 Jy_v10 Jz_v10]);
plane_mode=ENUM_plane_mode.V1000;
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

Lux=310/1000;
Luy=210/1000;
Ldx=205/1000;
Ldy=355.5/1000;
Lu=hypot(Lux,Luy);
Ld=hypot(Ldx,Ldy);
Ku=32;
Kd=Ku*Luy/Ldy;
Kx=(Ku-Kd)/(Ku+Kd);
Qd=10;
Qu=asind(Kd*sind(Qd)*Ld/(Ku*Lu));
Kc=(Ku*Lux)/(Ldx*Kd);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  ts=0;
  GRAVITY_MSS=9.80665;
  HD=180/pi;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BUS_CTRL_Curr_sate.pitch=0;
BUS_CTRL_Curr_sate.roll=0;
BUS_CTRL_Curr_sate.yaw=0;

BUS_CTRL_Curr_sate.gyro_x=0;
BUS_CTRL_Curr_sate.gyro_y=0;
BUS_CTRL_Curr_sate.gyro_z=0;

BUS_CTRL_Curr_sate.accel_x=0;
BUS_CTRL_Curr_sate.accel_y=0;    
BUS_CTRL_Curr_sate.accel_z=0;

BUS_CTRL_Curr_sate.VN=0;
BUS_CTRL_Curr_sate.VE=0;  
BUS_CTRL_Curr_sate.Vz=0; 

BUS_CTRL_Curr_sate.aspeed=17;
BUS_CTRL_Curr_sate.EAS2TAS=1;  

BUS_CTRL_Curr_sate.curr_vel=[0 0 0];
BUS_CTRL_Curr_sate.curr_pos=[0 0 ];
BUS_CTRL_Curr_sate.groundspeed_vector=[0.1 0.1];
BUS_CTRL_Curr_sate.height=0;
BUS_CTRL_Curr_sate.curr_alt=0;
BUS_CTRL_Curr_sate.loc_origin=[40,100]*1e7;
BUS_CTRL_Curr_sate.current_loc=[40,100]*1e7;
BUS_CTRL_Curr_sate.center_WP=[40,100]*1e7;
BUS_CTRL_Curr_sate.prev_WP=[40,100]*1e7;
BUS_CTRL_Curr_sate.next_WP=[40,100.01]*1e7;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%sl
Custom_design.mode=0;
Custom_design.mode_state=0;
Custom_design.dt=0.012;
Custom_design.thrust_slew_time=0.3;%%%%%%%%%%%%%%%%油门时间
Custom_design.throttle_slewrate=100;%%%%%%%%%%%%%%%%TECS 油门变化率
Custom_design.current_tilt=0;%0.69
Custom_design.p_tail_tilt=1;
Custom_design.p_tilt_pitch_target=0.2;
    
Custom_design.aspeed_c2p=8;
Custom_design.aspeed_c2ps=13;
Custom_design.tail_tilt_c2p=-2800;
Custom_design.tail_tilt_p2c=-1300;
Custom_design.tail_tilt_rate=1500;
Custom_design.p_plane_c2p=0.1;
Custom_design.yaw_max_c2p=0.3;
Custom_design.p_plane_cp=0.4;
Custom_design.pitch_target_p2c=500;
Custom_design.k_throttle_c2p=0.2;
Custom_design.throttle_off_rate= 1; 
Custom_design.POSCONTROL_ACC_Z_FILT_HZ_c2p=1;
Custom_design.pitch_target_c2p= 0;

Custom_design.thr_out_min=0.20;
Custom_design.throttle_ground=0.55;
Custom_design.aspeed_cp=30;
Custom_design.yaw_in_max=0.1;
Custom_design.imu_filt=10;
Custom_design.roll_target_pilot=0;
Custom_design.pitch_target_pilot=0;
Custom_design.take_off_land=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PathModeOut_sl.headingCmd=0;
PathModeOut_sl.groundspeedCmd=0;
PathModeOut_sl.heightCmd=10000;
PathModeOut_sl.flightTaskMode=ENUM_FlightTaskMode.LandMode;
PathModeOut_sl.flightControlMode=ENUM_FlightControlMode.SpotHoverMode;
PathModeOut_sl.maxClimbSpeed=100;
PathModeOut_sl.turnCenterLL=[40,100]*1e7;
PathModeOut_sl.prePathPoint_LLA=[40,100 ,0]*1e7;
PathModeOut_sl.curPathPoint_LLA=[40,100.01, 0]*1e7;
PathModeOut_sl.rollCmd=0.1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Forward_Throttle.vel_forward_last_pct=0;
Forward_Throttle.vel_forward_gain=1;
Forward_Throttle.vel_forward_min_pitch=-5;
Forward_Throttle.vel_forward_tail_tilt_max=2000;
Forward_Throttle.vel_forward_integrator=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% wind %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5    
WeatherVane.weathervane_min_roll=8;
WeatherVane.weathervane_gain=0.01;       
WeatherVane.yaw_rate_max=50;
WeatherVane.weathervane_last_output=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%AC_AttitudeControl matlab sim  
AC_AttitudeControl.slew_yaw=6000;
AC_AttitudeControl.accel_yaw_max=18000;
AC_AttitudeControl.rate_bf_ff_enabled=1;
AC_AttitudeControl.accel_roll_max=72000;
AC_AttitudeControl.accel_pitch_max=30000; 
AC_AttitudeControl.angle_boost_enabled=1;
AC_AttitudeControl.angle_limit_tc=1;
AC_AttitudeControl.ang_vel_roll_max=0;
AC_AttitudeControl.ang_vel_pitch_max=360;
AC_AttitudeControl.ang_vel_yaw_max=0;
AC_AttitudeControl.input_tc=0.3;
AC_AttitudeControl.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS= radians(40.0) ;    %minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_AttitudeControl.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS= radians(720.0);   %maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_AttitudeControl.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS= radians(10.0) ;   % minimum body-frame acceleration limit for the stability controller (for yaw axis)
AC_AttitudeControl.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS=radians(120.0) ;   % maximum body-frame acceleration limit for the stability controller (for yaw axis)
AC_AttitudeControl.AC_ATTITUDE_THRUST_ERROR_ANGLE=radians(30.0);               %Thrust angle error above which yaw corrections are limited
AC_AttitudeControl.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX=0.8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AC_AttitudeControl.ATC_ACCEL_P_MAX = 36397.976562;
AC_AttitudeControl.ATC_ACCEL_R_MAX = 33111.574219;
AC_AttitudeControl.ATC_ACCEL_Y_MAX = 36397.976562;

AC_AttitudeControl.p_angle_roll=3;
AC_AttitudeControl.p_angle_pitch=3;
AC_AttitudeControl.p_angle_yaw=3;
AC_AttitudeControl.ATC_RAT_PIT_D =   0.00;
AC_AttitudeControl.ATC_RAT_PIT_FF =  0.000000;
AC_AttitudeControl.ATC_RAT_PIT_FILT =20.000000;
AC_AttitudeControl.ATC_RAT_PIT_I   = 0.0;
AC_AttitudeControl.ATC_RAT_PIT_I_inint=0.1;
AC_AttitudeControl.ATC_RAT_PIT_IMAX =0.1;
AC_AttitudeControl.ATC_RAT_PIT_P   = 0.20;

AC_AttitudeControl.ATC_RAT_RLL_D  =  0;
AC_AttitudeControl.ATC_RAT_RLL_FF =  0.000000;
AC_AttitudeControl.ATC_RAT_RLL_FILT= 20.000000;
AC_AttitudeControl.ATC_RAT_RLL_I    =0.0;
AC_AttitudeControl.ATC_RAT_RLL_I_inint=0;
AC_AttitudeControl.ATC_RAT_RLL_IMAX =0.0;
AC_AttitudeControl.ATC_RAT_RLL_P    =0.1;

AC_AttitudeControl.ATC_RAT_YAW_D    =0.00000;
AC_AttitudeControl.ATC_RAT_YAW_FF  = 0.000000;
AC_AttitudeControl.ATC_RAT_YAW_FILT =5.000000;
AC_AttitudeControl.ATC_RAT_YAW_I    =0.0;
AC_AttitudeControl.ATC_RAT_YAW_I_inint=0;
AC_AttitudeControl.ATC_RAT_YAW_IMAX =0.0;
AC_AttitudeControl.ATC_RAT_YAW_P    =0.1;

AC_AttitudeControl.rate_yaw_pid_reset_filter=0;
AC_AttitudeControl.rate_yaw_pid_input=0;
AC_AttitudeControl.rate_yaw_pid_derivative=0;
AC_AttitudeControl.rate_yaw_pid_integrator=0;
AC_AttitudeControl.motors_limit_yaw=0;

AC_AttitudeControl.rate_roll_pid_reset_filter=0;
AC_AttitudeControl.rate_roll_pid_input=0;
AC_AttitudeControl.rate_roll_pid_derivative=0;
AC_AttitudeControl.rate_roll_pid_integrator=0;
AC_AttitudeControl.motors_limit_roll_pitch=0;

AC_AttitudeControl.rate_pitch_pid_reset_filter=0;
AC_AttitudeControl.rate_pitch_pid_input=0;
AC_AttitudeControl.rate_pitch_pid_derivative=0;
AC_AttitudeControl.rate_pitch_pid_integrator=0;
AC_AttitudeControl.rot_body_to_ned=eye(3,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AC_PosControl.POSCONTROL_POS_Z_P=3;    % vertical position controller P gain default
AC_PosControl.POSCONTROL_VEL_Z_P=1;    % vertical velocity controller P gain default
AC_PosControl.POSCONTROL_ACC_Z_P=1;    % vertical acceleration controller P gain default
AC_PosControl.POSCONTROL_ACC_Z_I=0.0;    % vertical acceleration controller I gain default
AC_PosControl.POSCONTROL_ACC_Z_I_inint=0.1;
AC_PosControl.POSCONTROL_ACC_Z_D=0.1;    % vertical acceleration controller D gain default
AC_PosControl.POSCONTROL_ACC_Z_IMAX=800;    % vertical acceleration controller IMAX gain default
AC_PosControl.POSCONTROL_ACC_Z_FILT_HZ=20.0;    % vertical acceleration controller input filter default
AC_PosControl.POSCONTROL_ACC_Z_FILT_HZ_inint=20;
AC_PosControl.POSCONTROL_ACC_Z_DT=0.0025;    % vertical acceleration controller dt default

AC_PosControl.POSCONTROL_POS_XY_P=0.8;    % horizontal position controller P gain default
AC_PosControl.POSCONTROL_VEL_XY_P=2;    % horizontal velocity controller P gain default
AC_PosControl.POSCONTROL_VEL_XY_I=0.1;    % horizontal velocity controller I gain default
AC_PosControl.POSCONTROL_VEL_XY_I_inint=0.1;
AC_PosControl.POSCONTROL_VEL_XY_D=0.0;    % horizontal velocity controller D gain default
AC_PosControl.POSCONTROL_VEL_XY_IMAX=1000.0;    % horizontal velocity controller IMAX gain default
AC_PosControl.POSCONTROL_VEL_XY_FILT_HZ=5.0;    % horizontal velocity controller input filter
AC_PosControl.POSCONTROL_VEL_XY_FILT_D_HZ=5.0;    % horizontal velocity controller input filter for D

AC_PosControl.POSCONTROL_ACCELERATION_MIN=50.0;% minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
AC_PosControl.POSCONTROL_ACCEL_XY=        100.0;%default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
AC_PosControl.POSCONTROL_ACCEL_XY_MAX=    357.0;%max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
AC_PosControl.POSCONTROL_STOPPING_DIST_UP_MAX=         300.0;%max stopping distance (in cm) vertically while climbing
AC_PosControl.POSCONTROL_STOPPING_DIST_DOWN_MAX=       200.0;%max stopping distance (in cm) vertically while descending

AC_PosControl.POSCONTROL_SPEED=           300.0;%default horizontal speed in cm/s
AC_PosControl.POSCONTROL_SPEED_DOWN=     -300.0;%default descent rate in cm/s
AC_PosControl.POSCONTROL_SPEED_UP=        300.0;%default climb rate in cm/s

AC_PosControl.POSCONTROL_ACCEL_Z=         250.0;%default vertical acceleration in cm/s/s.

AC_PosControl.POSCONTROL_LEASH_LENGTH_MIN=100.0;%minimum leash lengths in cm

AC_PosControl.POSCONTROL_DT_50HZ=         0.02;% time difference in seconds for 50hz update rate
AC_PosControl.POSCONTROL_DT_400HZ=        0.0025;% time difference in seconds for 400hz update rate

AC_PosControl.POSCONTROL_ACTIVE_TIMEOUT_US=200000 ;% position controller is considered active if it has been called within the past 0.2 seconds

AC_PosControl.POSCONTROL_VEL_ERROR_CUTOFF_FREQ=4.0;% low-pass filter on velocity error (unit: hz)
AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=2.0;% low-pass filter on accel error (unit: hz)
AC_PosControl.POSCONTROL_ACCEL_FILTER_HZ= 2.0;% low-pass filter on acceleration (unit: hz)
AC_PosControl.POSCONTROL_JERK_RATIO=      1.0;% Defines the time it takes to reach the requested acceleration

AC_PosControl.POSCONTROL_OVERSPEED_GAIN_Z=2.0;% gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range
AC_PosControl.accel_xy_filt_hz=AC_PosControl.POSCONTROL_ACCEL_FILTER_HZ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AC_PosControl.recalc_leash_z=1;
AC_PosControl.limit_pos_down=0;
AC_PosControl.limit_pos_up=0;
AC_PosControl.pos_error=[0 0 0];
AC_PosControl.pos_target=[0 0 0];
AC_PosControl.vel_target=[0 0 0];
AC_PosControl.limit_vel_up=0;
AC_PosControl.limit_vel_down=0;
AC_PosControl.use_desvel_ff_z=1;
AC_PosControl.reset_rate_to_accel_z=1;
AC_PosControl.vel_desired=[0 0 0];
AC_PosControl.vel_last=[0 0 0];
AC_PosControl.accel_desired=[0 0 0];
AC_PosControl.freeze_ff_z=0;
AC_PosControl.vel_error=[0 0 0];
AC_PosControl.vel_error_input=0;
AC_PosControl.accel_target=[0 0 0];
AC_PosControl.reset_accel_to_throttle=1;
AC_PosControl.accel_error=[0 0 0];
AC_PosControl.z_accel_meas=0;
AC_PosControl.pid_accel_z_reset_filter=1;
AC_PosControl.pid_accel_z_input=0;
AC_PosControl.pid_accel_z_derivative=0;
AC_PosControl.pid_accel_z_integrator=0;
AC_PosControl.throttle_lower=0;
AC_PosControl.throttle_upper=0;
AC_PosControl.throttle_input=0;
%%%%%%%%%%%%%%%%%%
AC_PosControl.roll_target=0;
AC_PosControl.pitch_target=0;
AC_PosControl.yaw_target=0;
AC_PosControl.target_yaw_rate=0;

AC_PosControl.attitude_target_euler_angle=[AC_PosControl.roll_target AC_PosControl.pitch_target AC_PosControl.yaw_target]*0.01/HD;
AC_PosControl.attitude_target_quat=from_euler(AC_PosControl.attitude_target_euler_angle);
AC_PosControl.attitude_target_ang_vel=[0 0 0];
AC_PosControl.attitude_target_euler_rate=[0 0 0];
AC_PosControl.attitude_ang_error=[1 0 0 0];
AC_PosControl.use_sqrt_controller=1;

AC_PosControl.throttle_rpy_mix=0;
AC_PosControl.pid_vel_xy_reset_filter=1;
AC_PosControl.pid_vel_xy_input=[0 0];
AC_PosControl.pid_vel_xy_derivative=[0 0];
AC_PosControl.pid_vel_xy_integrator=[0 0];
AC_PosControl.limit_accel_xy=0;
AC_PosControl.motors_limit_throttle_upper=0;
AC_PosControl.reset_accel_to_lean_xy=1;
AC_PosControl.accel_xy_input=[0 0];
AC_PosControl.accel_xy_angle_max=2000;%%dai cha
AC_PosControl.recalc_leash_xy=1;
AC_PosControl.reset_desired_vel_to_pos=1;
AC_PosControl.accel_last_z_cms=0;
AC_PosControl.is_active_z=1;
AC_PosControl.is_active_xy=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AP_MotorsMulticopter.inint=1;
AP_MotorsMulticopter.AP_MOTORS_MATRIX_YAW_FACTOR_CW=-1;
AP_MotorsMulticopter.AP_MOTORS_MATRIX_YAW_FACTOR_CCW=1;
AP_MotorsMulticopter.throttle_hover=0.48;
AP_MotorsMulticopter.throttle_cruise=15;
AP_MotorsMulticopter.p_ff_throttle=0.35;
AP_MotorsMulticopter.pwm_max=2000;
AP_MotorsMulticopter.pwm_min=1000;
AP_MotorsMulticopter.pwm_out=[1000 1000 1000 1000]+AP_MotorsMulticopter.throttle_hover*1000;
AP_MotorsMulticopter.pwm_tail=1000;
AP_MotorsMulticopter.tail_tilt=0;

AP_MotorsMulticopter.roll_factor=[0 0 0 0];
AP_MotorsMulticopter.pitch_factor=[0 0 0 0];
AP_MotorsMulticopter.yaw_factor=[0 0 0 0];

AP_MotorsMulticopter.limit_roll_pitch=0;
AP_MotorsMulticopter.limit_yaw=0;
AP_MotorsMulticopter.yaw_headroom=0;
AP_MotorsMulticopter.air_density_ratio=1;
AP_MotorsMulticopter.thrust_boost=0;

AP_MotorsMulticopter.angle_boost=0;
AP_MotorsMulticopter.throttle_avg_max=1;
AP_MotorsMulticopter.throttle_cutoff_frequency=1;
AP_MotorsMulticopter.throttle_thrust_max=1;
AP_MotorsMulticopter.althold_lean_angle_max=0;
AP_MotorsMulticopter.throttle_rpy_mix_desired=0;
AP_MotorsMulticopter.AC_ATTITUDE_CONTROL_MAX=5;
AP_MotorsMulticopter.roll_in=0;
AP_MotorsMulticopter.pitch_in=0;
AP_MotorsMulticopter.yaw_in=0;
AP_MotorsMulticopter.armed=1;
AP_MotorsMulticopter.throttle_in=0.4816;
AP_MotorsMulticopter.throttle_filter=0.4816;
AP_MotorsMulticopter.thrust_boost_ratio=0;
AP_MotorsMulticopter.thrust_rpyt_out=[0 0 0 0];
AP_MotorsMulticopter.thrust_error_angle=0;
AP_MotorsMulticopter.rate_target_ang_vel=[0 0 0];
AP_MotorsMulticopter.attitude_error_vector=[0 0 0];
AP_MotorsMulticopter.climb_rate_cms=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AP_PitchController
AP_PitchController.gains_tau_pitch=0.5;
AP_PitchController.gains_P_pitch=3;
AP_PitchController.gains_D_pitch=0.1;
AP_PitchController.gains_I_pitch=0.1;
AP_PitchController.gains_rmax_pitch=0;
AP_PitchController.max_rate_neg=0;
AP_PitchController.roll_ff_pitch=1;
AP_PitchController.roll_ff_pitch_inint=1;
AP_PitchController.gains_imax_pitch=3000;
AP_PitchController.kff_throttle_to_pitch=0;
AP_PitchController.disable_integrator_pitch=0;
AP_PitchController.desired_rate_pitch=0;
AP_PitchController.nav_pitch_cd=0;

AP_PitchController.last_out_pitch=0;
AP_PitchController.pid_info_I_pitch=0;
AP_PitchController.pid_info_P_pitch=0;
AP_PitchController.pid_info_FF_pitch=0;
AP_PitchController.pid_info_D_pitch=0;
AP_PitchController.pid_info_desired_pitch=0;
AP_PitchController.pid_info_actual_pitch=0;
AP_PitchController.gains_FF_pitch=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_RollController
AP_RollController.gains_tau_roll=0.5;
AP_RollController.gains_P_roll=1;
AP_RollController.gains_D_roll=0.05;
AP_RollController.gains_I_roll=0.1;
AP_RollController.gains_rmax_roll=0;
AP_RollController.gains_imax_roll=3000;
AP_RollController.nav_roll_cd=0;
AP_RollController.inverted_flight=0;

AP_RollController.gains_FF_roll=0;
AP_RollController.disable_integrator_roll=0;
AP_RollController.desired_rate_roll=0;
AP_RollController.last_out_roll=0;
AP_RollController.pid_info_I_roll=0;
AP_RollController.pid_info_P_roll=0;
AP_RollController.pid_info_FF_roll=0;
AP_RollController.pid_info_D_roll=0;
AP_RollController.pid_info_desired_roll=0;
AP_RollController.pid_info_actual_roll=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%  AP_YawController 
AP_YawController.K_A_yaw=0;
AP_YawController.K_I_yaw=1;
AP_YawController.K_D_yaw=0.5;
AP_YawController.K_FF_yaw=1;
AP_YawController.K_FF_yaw_inint=1;     
AP_YawController.imax_yaw=1500;  
AP_YawController.disable_integrator_yaw=0;
AP_YawController.kff_rudder_mix=0;

AP_YawController.K_D_last_yaw=0;
AP_YawController.integrator_yaw=0;
AP_YawController.last_out_yaw=0;
AP_YawController.pid_info_I_yaw=0;
AP_YawController.pid_info_D_yaw=0;
AP_YawController.last_rate_hp_out_yaw=0;
AP_YawController.last_rate_hp_in_yaw=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_TECS
AP_TECS.maxClimbRate=5;
AP_TECS.minSinkRate=3;
AP_TECS.timeConstant=5;%timeConstant
AP_TECS.thrDamp=0.7;
AP_TECS.integGain=0.1;
AP_TECS.vertAccLim=5;
AP_TECS.spdCompFiltOmega=2;
AP_TECS.rollComp=0;
AP_TECS.spdWeight=1;
AP_TECS.ptchDamp=0.7;
AP_TECS.maxSinkRate=7;
AP_TECS.EAS_dem=17;
AP_TECS.TAS_dem=17;
AP_TECS.TAS_dem_adj=AP_TECS.TAS_dem;
AP_TECS.TAS_state=17;
AP_TECS.integDTAS_state=0;

AP_TECS.highest_airspeed=35;
AP_TECS.scaling_speed=17;
AP_TECS.airspeed_max=25;
AP_TECS.airspeed_min=13;

AP_TECS.throttle_min=0;
AP_TECS.throttle_max=100; 
AP_TECS.pitch_limit_min_cd=-1500;
AP_TECS.pitch_limit_max_cd=2000;
AP_TECS.pitch_max_limit=90;
AP_TECS.roll_limit_cd=2500;
AP_TECS.roll_limit_cd_inint=4500;
AP_TECS.inint_hgt=1;

AP_TECS.TASmax=19;
AP_TECS.TASmin=14;
AP_TECS.THRmaxf=1;
AP_TECS.THRminf=0;
AP_TECS.EAS_dem_cm=1700;
AP_TECS.hgt_dem_cm=0;
AP_TECS.arspeed_filt=35;
AP_TECS.arspeed_temp=0;

AP_TECS.k_rudder=0;
AP_TECS.k_aileron=0;
AP_TECS.k_throttle=0.0;
AP_TECS.k_elevator=0;

AP_TECS.vdot_filter=zeros(5,1);
AP_TECS.smoothed_airspeed=0;
AP_TECS.aerodynamic_load_factor=1;
AP_TECS.pitch_max=0;
AP_TECS.pitch_min=0;   
AP_TECS.SKE_dem=0;
AP_TECS.SPE_dem=0;
AP_TECS.SPE_est=0;
AP_TECS.SKE_est=0;
AP_TECS.STE_error=0;
AP_TECS.SPEdot_dem=0;
AP_TECS.SKEdot_dem=0;
AP_TECS.STEdot_min=0;
AP_TECS.STEdot_max=0;
AP_TECS.SPEdot=0;
AP_TECS.SKEdot=0;
AP_TECS.STEdotErrLast=0;
AP_TECS.throttle_dem=0;
AP_TECS.last_throttle_dem=0;
AP_TECS.integTHR_state=0;
AP_TECS.hgt_dem=0;
AP_TECS.PITCHmaxf=0;
AP_TECS.PITCHminf=0;
AP_TECS.TAS_rate_dem=0;
AP_TECS.hgt_dem_in_old=0;
AP_TECS.max_sink_rate=0;
AP_TECS.hgt_dem_prev=0;
AP_TECS.hgt_dem_adj=0;
AP_TECS.hgt_dem_adj_last=0;
AP_TECS.hgt_rate_dem=0;
AP_TECS.climb_rate=0;
AP_TECS.pitch_dem=0;
AP_TECS.integSEB_state=0;
AP_TECS.pitch_dem_unc=0;
AP_TECS.last_pitch_dem=0;
AP_TECS.vel_dot=0;
AP_TECS.ff_throttle=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_L1_Control
AP_L1_Control.L1_period=17;
AP_L1_Control.L1_damping=0.75;
AP_L1_Control.L1_xtrack_i_gain=0.00;
AP_L1_Control.loiter_bank_limit=0;
AP_L1_Control.LOCATION_SCALING_FACTOR = 0.011131884502145034;
AP_L1_Control.LOCATION_SCALING_FACTOR_INV = 89.83204953368922;
AP_L1_Control.radius=60;
AP_L1_Control.L1_radius=30;
AP_L1_Control.loiter_direction=1;
AP_L1_Control.mode_L1=0;
AP_L1_Control.dist_min=50;
        
AP_L1_Control.loc.num=[0     1     2     3     4      5	99  99  99  99  99];
AP_L1_Control.loc.lat=[4.0000    4.0010    4.0020    4.0020    4.0010  4.0015     4.0010  0   0   0   0]*1e8;
AP_L1_Control.loc.lon=[1.0000    1.0001    1.0001    1.0002    1.0002  1.00025    1.0001  0   0   0	0]*1e9;

AP_L1_Control.target_bearing_cd=0;
AP_L1_Control.L1_dist=0;
AP_L1_Control.crosstrack_error=0;
AP_L1_Control.nav_bearing=0;
AP_L1_Control.L1_xtrack_i_gain_prev=0;
AP_L1_Control.L1_xtrack_i=0;
AP_L1_Control.last_Nu=0;
AP_L1_Control.latAccDem=0;
AP_L1_Control.WPcircle=0;
AP_L1_Control.bearing_error=0;
AP_L1_Control.data_is_stale=0;
AP_L1_Control.reverse=0;



