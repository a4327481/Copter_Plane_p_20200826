function auto_mode()
%auto flight
%mode auto
global dt
global HD
global plane_mode
global Plane
global PathModeOut_sl
global AC_PosControl
global AP_Motors
global AP_L1
global AP_TECS
global AC_rate_pitch_pid
global AC_rate_roll_pid
global AC_rate_yaw_pid
global SRV_Channel
global Copter_Plane
global SINS

radius                             = Copter_Plane.radius;
loiter_direction                   = Copter_Plane.loiter_direction;
p_tilt_pitch_target                = Copter_Plane.p_tilt_pitch_target;
dist_min                           = Copter_Plane.dist_min;
p_plane_c2p                        = Copter_Plane.p_plane_c2p;
yaw_max_c2p                        = Copter_Plane.yaw_max_c2p;
POSCONTROL_ACC_Z_FILT_HZ_c2p       = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p;
aspeed_c2p                         = Copter_Plane.aspeed_c2p;
aspeed_c2ps                        = Copter_Plane.aspeed_c2ps;
pitch_target_c2p                   = Copter_Plane.pitch_target_c2p;
k_throttle_c2p                     = Copter_Plane.k_throttle_c2p;
tail_tilt_c2p                      = Copter_Plane.tail_tilt_c2p;
tail_tilt_rate                     = Copter_Plane.tail_tilt_rate;
pitch_target_p2c                   = Copter_Plane.pitch_target_p2c;
POSCONTROL_ACC_Z_FILT_HZ           = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ;
POSCONTROL_THROTTLE_CUTOFF_FREQ    = Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ;
POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c= Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c;
throttle_ground                    = Copter_Plane.throttle_ground;
throttle_off_rate                  = Copter_Plane.throttle_off_rate;
k_flap_TakeOff                     = Copter_Plane.k_flap_TakeOff;
k_flap_Land                        = Copter_Plane.k_flap_Land;
k_flap                             = Copter_Plane.k_flap;
Fix2Rotor_delay_s                  = Copter_Plane.Fix2Rotor_delay_s;
heading_hold                       = Copter_Plane.heading_hold;
thr_out_min                        = Copter_Plane.thr_out_min;
throttle_filter                    = AP_Motors.throttle_filter;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
height                                = SINS.curr_alt/100;
aspeed                                = SINS.aspeed;
curr_loc                              = SINS.curr_loc;
curr_alt                              = SINS.curr_alt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
persistent WP_i
persistent PathMode
persistent Rotor2Fix_delay
persistent Rotor2Fix_delay_flag
persistent Fix2Rotor_delay
persistent Fix2Rotor_delay_flag
persistent TakeOffMode_delay
persistent TakeOffMode_delay_flag
persistent Land_throttle_in
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isempty(WP_i)
    WP_i = 1;
end
if isempty(PathMode)
    PathMode = ENUM_FlightTaskMode.NoneFlightTaskMode;
end
if isempty(Rotor2Fix_delay)
    Rotor2Fix_delay = 0;
end
if isempty(Rotor2Fix_delay_flag)
    Rotor2Fix_delay_flag = false;
end

if isempty(Fix2Rotor_delay)
    Fix2Rotor_delay = 0;
end
if isempty(Fix2Rotor_delay_flag)
    Fix2Rotor_delay_flag = 0;
end
if isempty(TakeOffMode_delay)
    TakeOffMode_delay = 0;
end
if isempty(TakeOffMode_delay_flag)
    TakeOffMode_delay_flag = 0;
end

if isempty(Land_throttle_in)
    Land_throttle_in = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
error_pos=8;
if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.TakeOffMode||PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.LandMode)
    Copter_Plane.take_off_land=1;
else
    Copter_Plane.take_off_land=0;
end

if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.TakeOffMode)
    SRV_Channel.k_flap=k_flap_TakeOff;
elseif(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.LandMode)
    SRV_Channel.k_flap=k_flap_Land;
else
    SRV_Channel.k_flap=k_flap;
end
if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.Rotor2Fix_Mode)
    AC_PosControl.thr_out_min=Copter_Plane.thr_out_min_c2p;
else
    AC_PosControl.thr_out_min=Copter_Plane.thr_out_min;
end

if(PathModeOut_sl.flightTaskMode~=ENUM_FlightTaskMode.Fix2Rotor_Mode)
    AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ;
end
if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.LandMode)
    Copter_Plane.disable_forward_throttle=true;
    AC_PosControl.pid_vel_xy_p_wind=(throttle_filter-thr_out_min-0.1)/0.1*dt/5 + AC_PosControl.pid_vel_xy_p_wind;
    AC_PosControl.pid_vel_xy_p_wind=constrain_value(AC_PosControl.pid_vel_xy_p_wind,0.5,1);
else
   Copter_Plane. disable_forward_throttle=false;
    AC_PosControl.pid_vel_xy_p_wind=1;
end

if( Copter_Plane.State==ENUM_State.E_Plane)%% disable plane I,vel_forward_integrator=0;
    Copter_Plane.disable_AP_roll_integrator           = false;
    Copter_Plane.disable_AP_pitch_integrator          = false;
    Copter_Plane.disable_AP_yaw_integrator            = false;
    Copter_Plane.disable_AP_rate_pitch_roll_ff        = false;
    Copter_Plane.disable_AP_rate_pitch_gains_D        = false;
    Copter_Plane.disable_AP_rate_yaw_K_FF             = false;
    Copter_Plane.disable_AP_rate_roll_gains_D         = false;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Copter_Plane.vel_forward_integrator               = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    AC_PosControl.pid_accel_z.flags_reset_filter      = true;
    AC_PosControl.pid_vel_xy.flags_reset_filter       = true;
    AC_rate_pitch_pid.flags_reset_filter              = true;
    AC_rate_roll_pid.flags_reset_filter               = true;
    AC_rate_yaw_pid.flags_reset_filter                = true;
    
    AC_PosControl.pid_accel_z.disable_integrator      = true;
    AC_PosControl.pid_vel_xy.disable_integrator       = true;
    AC_rate_pitch_pid.disable_integrator              = true;
    AC_rate_roll_pid.disable_integrator               = true;
    AC_rate_yaw_pid.disable_integrator                = true;
else
    Copter_Plane.disable_AP_roll_integrator           = true;
    Copter_Plane.disable_AP_pitch_integrator          = true;
    Copter_Plane.disable_AP_yaw_integrator            = true;
    Copter_Plane.disable_AP_rate_pitch_roll_ff        = true;
    Copter_Plane.disable_AP_rate_pitch_gains_D        = true;
    Copter_Plane.disable_AP_rate_yaw_K_FF             = true;
    Copter_Plane.disable_AP_rate_roll_gains_D         = true;
    
    AC_PosControl.pid_accel_z.disable_integrator      = false;
    AC_PosControl.pid_vel_xy.disable_integrator       = false;
    AC_rate_pitch_pid.disable_integrator              = false;
    AC_rate_roll_pid.disable_integrator               = false;
    AC_rate_yaw_pid.disable_integrator                = false;
end
AC_PosControl.pid_accel_z.filt_E_hz             = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ;
AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ   = Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch PathModeOut_sl.flightTaskMode
    case    ENUM_FlightTaskMode.TakeOffMode
        if((PathMode==ENUM_FlightTaskMode.GroundStandByMode)&&(TakeOffMode_delay_flag==0))
            Copter_Plane.State=ENUM_State.E_Copter;
            TakeOffMode_delay=TakeOffMode_delay+dt;
            init_vel_controller_xyz();
            set_Plane_SRV_to_zero();
            relax_attitude_controllers();
            rate_controller_run();
            set_throttle_out( 0.1,false, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
            AP_Motors_output();
            if(TakeOffMode_delay>1)
                TakeOffMode_delay_flag=1;
            end
        else
            if(PathMode~=ENUM_FlightTaskMode.TakeOffMode)
                PathMode=ENUM_FlightTaskMode.TakeOffMode;
                Copter_Plane.State=ENUM_State.E_Copter;
                Copter_Plane.loc_origin=curr_loc;
                SINS.curr_pos(1:2)=[0 0];
                init_vel_controller_xyz_c2c();
                AC_PosControl.pos_target(3)=max(curr_alt,10);
                relax_attitude_controllers_c2c();
            end
            SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,Copter_Plane.loc_origin)*100;   
            if(curr_alt<100)
                AC_PosControl.pid_accel_z.disable_integrator=true;
                AC_PosControl.pid_vel_xy.disable_integrator=true;
                AC_rate_pitch_pid.disable_integrator=true;
                AC_rate_roll_pid.disable_integrator=true;
                AC_rate_yaw_pid.disable_integrator=true;
            end
            climb_rate_cms = PathModeOut_sl.maxClimbSpeed;
            Copter_Plane.climb_rate_cms = climb_rate_cms;
%             
            if(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.RotorGoUpDownBySpeed)
                Copter_Plane.take_off_land=1;
                set_alt_target_from_climb_rate_ffg1(climb_rate_cms, dt, 0,1);
                copter_run();
                AC_PosControl.pos_target(3)  = PathModeOut_sl.heightCmd;
            elseif(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.SpotHoverMode)
                Copter_Plane.take_off_land=0;
                set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);
                copter_run();
            else
                Copter_Plane.take_off_land=0;
                set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);
                copter_run();
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.LandMode
        if(PathMode~=ENUM_FlightTaskMode.LandMode)
            PathMode=ENUM_FlightTaskMode.LandMode;
            Copter_Plane.State=ENUM_State.E_Copter;
            loc_origin=PathModeOut_sl.turnCenterLL(1:2);
            SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;
            init_vel_controller_xyz_c2c();
            relax_attitude_controllers_c2c();
        else
            AC_PosControl.pos_target(1:2)=[0 0];
            loc_origin=PathModeOut_sl.turnCenterLL(1:2);
            SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;
        end
        climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
        Copter_Plane.climb_rate_cms = climb_rate_cms;
        set_alt_target_from_climb_rate_ffg1(climb_rate_cms, dt, 0,-1);
        if(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.RotorGoUpDownWithHorizonPosFree)
            copter_run_posfree();
        else
            copter_run();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.HoverAdjustMode
        if(PathMode~=ENUM_FlightTaskMode.HoverAdjustMode)
            PathMode=ENUM_FlightTaskMode.HoverAdjustMode;
            Copter_Plane.State=ENUM_State.E_Copter;
            loc_origin=PathModeOut_sl.turnCenterLL(1:2);
            SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;
            init_vel_controller_xyz_c2c();
            relax_attitude_controllers_c2c();
        end
        AC_PosControl.pos_target(1:2)=[0 0];
        loc_origin=PathModeOut_sl.turnCenterLL(1:2);
        SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;
        Copter_Plane.climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
        set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,Copter_Plane.climb_rate_cms, dt, 0);
        copter_run();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.HoverUpMode
        if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
            PathMode=ENUM_FlightTaskMode.HoverUpMode;
            Copter_Plane.State=ENUM_State.E_Plane;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init_p2p();
        end
        hgt_dem_cm = Copter_Plane.hgt_dem_cm;
        if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos
            climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
            climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        else
            climb_rate_cms=0;
            hgt_dem_cm=PathModeOut_sl.heightCmd;
        end
        Copter_Plane.climb_rate_cms          = climb_rate_cms;
        Copter_Plane.hgt_dem_cm = hgt_dem_cm;
        center_WP=PathModeOut_sl.turnCenterLL(1:2);
        update_loiter( center_WP,   radius,   loiter_direction)
        plane_run();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.HoverDownMode
        if(PathMode~=ENUM_FlightTaskMode.HoverDownMode)
            PathMode=ENUM_FlightTaskMode.HoverDownMode;
            Copter_Plane.State=ENUM_State.E_Plane;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init_p2p();
        end
        center_WP=PathModeOut_sl.turnCenterLL(1:2);
        update_loiter( center_WP,   radius,   loiter_direction)
        if(PathModeOut_sl.rollCmd==0)
            AP_L1.latAccDem=0;
        end
        if(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.HoverSliderMode)
            AP_TECS.spdWeight=2;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            plane_run();
        else
            hgt_dem_cm = Copter_Plane.hgt_dem_cm;
            if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos
                climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
            elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
            else
                climb_rate_cms=0;
                hgt_dem_cm=PathModeOut_sl.heightCmd;
            end
            Copter_Plane.climb_rate_cms  = climb_rate_cms;
            Copter_Plane.hgt_dem_cm      = hgt_dem_cm;
            plane_run();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.CalibBeforePath
        if(PathMode~=ENUM_FlightTaskMode.CalibBeforePath)
            PathMode=ENUM_FlightTaskMode.CalibBeforePath;
            Copter_Plane.State=ENUM_State.E_Plane;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init_p2p();
        end
        hgt_dem_cm = Copter_Plane.hgt_dem_cm;
        if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos
            climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
            climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        else
            climb_rate_cms=0;
            hgt_dem_cm=PathModeOut_sl.heightCmd;
        end
        Copter_Plane.hgt_dem_cm = hgt_dem_cm;
        Copter_Plane.climb_rate_cms  = climb_rate_cms;
        center_WP=PathModeOut_sl.turnCenterLL(1:2);
        if(PathModeOut_sl.rollCmd>=0)
            Copter_Plane.loiter_direction=1;
        else
            Copter_Plane.loiter_direction=-1;
        end
        update_loiter( center_WP,   radius,   loiter_direction)
        plane_run();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.AirStandByMode
        if(PathMode~=ENUM_FlightTaskMode.AirStandByMode)
            PathMode=ENUM_FlightTaskMode.AirStandByMode;
            Copter_Plane.State=ENUM_State.E_Plane;
%             hgt_dem_cm=height*100;
%             Copter_Plane.hgt_dem_cm = hgt_dem_cm;
%             AP_TECS_init_p2p();
        end
        hgt_dem_cm = Copter_Plane.hgt_dem_cm;
        if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos
            climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
            climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        else
            climb_rate_cms=0;
            hgt_dem_cm=PathModeOut_sl.heightCmd;
        end
        Copter_Plane.hgt_dem_cm = hgt_dem_cm;
        Copter_Plane.climb_rate_cms  = climb_rate_cms;
        center_WP=PathModeOut_sl.turnCenterLL(1:2);
        update_loiter( center_WP,   radius,   loiter_direction)
        plane_run();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.Rotor2Fix_Mode
        switch plane_mode
            case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1,ENUM_plane_mode.V10s}
                if(PathMode~=ENUM_FlightTaskMode.Rotor2Fix_Mode)
                    PathMode=ENUM_FlightTaskMode.Rotor2Fix_Mode;
                    Copter_Plane.State=ENUM_State.E_Copter;
                    hgt_dem_cm=height*100;
                    Copter_Plane.hgt_dem_cm = hgt_dem_cm;
                    AP_TECS_init();
                    SINS.curr_pos(1:2)=[0 0];
                    init_vel_controller_xyz_c2c();
                    relax_attitude_controllers_c2c();
                    Rotor2Fix_delay_flag=false;
                    Rotor2Fix_delay=0;
                end
                if(Rotor2Fix_delay_flag)
                    if(Copter_Plane.State==ENUM_State.E_Plane)
                        Rotor2Fix_delay_flag=true;
                        Plane.nav_roll_cd=0; 
                        Plane.nav_pitch_cd= Copter_Plane.pitch_target_c2p;                      
                        SRV_Channel.k_throttle=AP_TECS.throttle_cruise * 0.01+k_throttle_c2p;
                        update_50hz();
                        AP_TECS_init()
                        stabilize();
                        AP_Motors_output();
                    else
%                         hgt_dem_cm=height*100;
%                         Copter_Plane.hgt_dem_cm = hgt_dem_cm;
                        SRV_Channel.k_throttle=AP_TECS.throttle_cruise * 0.01+k_throttle_c2p;
                        AP_TECS_init();
                        update_speed( Plane.aerodynamic_load_factor)
                        roll_target =  0;
                        pitch_target = 0/HD;
                        target_yaw_rate =0 ;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        rate_controller_run();
                        AC_PosControl.pitch_target = pitch_target;
                        AC_PosControl.roll_target  = roll_target;
                        %%%%
                        Plane.nav_pitch_cd=pitch_target;
                        Plane.nav_roll_cd=roll_target;
                        stabilize()
                        set_throttle_out( 0,false, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
                        AP_Motors_output();
                        if(AP_Motors.throttle_filter<0.1)
                            Copter_Plane.State=ENUM_State.E_Plane;
                            AP_TECS.throttle_dem=SRV_Channel.k_throttle;
                            AP_TECS.last_throttle_dem=SRV_Channel.k_throttle;
                        end
                    end
                else
                    SRV_Channel.k_throttle=AP_TECS.throttle_cruise * 0.01+k_throttle_c2p;
                    if(aspeed>aspeed_c2ps)
                        %%%%
                        roll_target =0 ;
                        pitch_target=0/HD;
                        target_yaw_rate =0;
                        AC_PosControl.pitch_target = pitch_target;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                        rate_controller_run();
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        update_z_controller();
                        AC_PosControl.pitch_target = pitch_target;
                        AC_PosControl.roll_target  = roll_target;
                        %%%%
                        Plane.nav_pitch_cd=pitch_target;
                        Plane.nav_roll_cd=roll_target;
                        stabilize()
                        AP_Motors_output();
                        Rotor2Fix_delay_flag=true;
                    elseif(aspeed>aspeed_c2p)
                        if(AC_PosControl.pitch_target>0)
                            AC_PosControl.pitch_target=0;
                        end
                        roll_target      = 0;
                        pitch_target_temp=AC_PosControl.pitch_target+p_tilt_pitch_target*-3000;
                        target_yaw_rate  = 0;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                        rate_controller_run();
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        update_z_controller();
                        %%%%
                        Plane.nav_pitch_cd=pitch_target_temp;
                        Plane.nav_roll_cd=roll_target;
                        stabilize()
                        SRV_Channel.k_aileron=SRV_Channel.k_aileron*p_plane_c2p;
                        SRV_Channel.k_elevator=SRV_Channel.k_elevator*p_plane_c2p;
                        SRV_Channel.k_rudder=SRV_Channel.k_rudder*p_plane_c2p;
                        AP_Motors_output();
                    else
                        if(AC_PosControl.pitch_target>0)
                            AC_PosControl.pitch_target=0;
                        end
                        roll_target       = 0;
                        pitch_target_temp = AC_PosControl.pitch_target+p_tilt_pitch_target*-3000;
                        target_yaw_rate   = 0;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                        rate_controller_run();
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                        update_z_controller();
                        SRV_Channel.k_aileron=0;
                        SRV_Channel.k_elevator=0;
                        SRV_Channel.k_rudder=0;
                        AP_Motors_output();
                    end
                end
                
            case {ENUM_plane_mode.V1000}
                if(PathMode~=ENUM_FlightTaskMode.Rotor2Fix_Mode)
                    PathMode=ENUM_FlightTaskMode.Rotor2Fix_Mode;
                    Copter_Plane.State=ENUM_State.E_Copter;
                    hgt_dem_cm=height*100;
                    Copter_Plane.hgt_dem_cm = hgt_dem_cm;
                    AP_TECS_init();
                    SINS.curr_pos(1:2)=[0 0];
                    init_vel_controller_xyz_c2c();
                    relax_attitude_controllers_c2c();
                    Rotor2Fix_delay_flag=false;
                    Rotor2Fix_delay=0;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                tail_error=tail_tilt_c2p-SRV_Channel.tail_tilt;
                tail_error=constrain_value(tail_error,-tail_tilt_rate*dt,tail_tilt_rate*dt);
                SRV_Channel.tail_tilt=SRV_Channel.tail_tilt+tail_error;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if(Rotor2Fix_delay_flag)
                    if(Copter_Plane.State==ENUM_State.E_Plane)
                        Rotor2Fix_delay_flag=true;
                        update_50hz();
                        Plane.nav_roll_cd=0;
                        Plane.nav_pitch_cd=pitch_target_c2p;
                        SRV_Channel.k_throttle=AP_TECS.throttle_cruise * 0.01+k_throttle_c2p;
                        stabilize()
                        SRV_Channel.k_elevator=SRV_Channel.k_elevator+Copter_Plane.p_k_elevator_c2p;
                        AP_Motors_output();
                    else
                        hgt_dem_cm=height*100;
                        Copter_Plane.hgt_dem_cm = hgt_dem_cm;
                        AP_TECS_init();
                        SRV_Channel.k_throttle=AP_TECS.throttle_cruise * 0.01+k_throttle_c2p;
                        roll_target =  0;
                        pitch_target = 3/HD;
                        target_yaw_rate =0 ;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
%                         AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        rate_controller_run();
                        %%%%
                        Plane.nav_pitch_cd=pitch_target;
                        Plane.nav_roll_cd=roll_target;
                        stabilize()
                        set_throttle_out( 0,false, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
                        AP_Motors_output();
                        if(AP_Motors.throttle_filter<0.1)
                            SRV_Channel.tail_tilt=-9556;
                            Rotor2Fix_delay=Rotor2Fix_delay+dt;
                            if(Rotor2Fix_delay>0.2)
                                Copter_Plane.State=ENUM_State.E_Plane;
                            end
                        end
                    end
                else
                    if(aspeed>aspeed_c2ps)
                        roll_target =0 ;
                        pitch_target=3/HD;
                        target_yaw_rate =0;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                        rate_controller_run();
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
%                         AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        update_z_controller();
                        %%%%
                        Plane.nav_pitch_cd=pitch_target;
                        Plane.nav_roll_cd=roll_target;
                        stabilize()
                        AP_Motors_output();
                        Rotor2Fix_delay_flag=true;
                    elseif(aspeed>aspeed_c2p)
                        tail_tilt_temp=constrain_value(SRV_Channel.tail_tilt,-3000,700);
                        if(AC_PosControl.pitch_target>0)
                            AC_PosControl.pitch_target=0;
                        end
                        roll_target      = 0;
                        pitch_target_temp=p_tilt_pitch_target*tail_tilt_temp;
                        target_yaw_rate  = 0;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                        rate_controller_run();
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
%                         AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        update_z_controller();
                        %%%%
                        Plane.nav_pitch_cd=pitch_target_temp;
                        Plane.nav_roll_cd=roll_target;
                        stabilize()
                        SRV_Channel.k_aileron=SRV_Channel.k_aileron*p_plane_c2p;
                        SRV_Channel.k_elevator=SRV_Channel.k_elevator*p_plane_c2p;
                        SRV_Channel.k_rudder=SRV_Channel.k_rudder*p_plane_c2p;
                        AP_Motors_output();
                        
                    else
                        tail_tilt_temp=constrain_value(SRV_Channel.tail_tilt,-3000,700);
                        if(AC_PosControl.pitch_target>0)
                            AC_PosControl.pitch_target=0;
                        end
                        roll_target       = 0;
                        pitch_target_temp = p_tilt_pitch_target*tail_tilt_temp;
                        target_yaw_rate   = 0;
                        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                        rate_controller_run();
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                        update_z_controller();
                        SRV_Channel.k_aileron=0;
                        SRV_Channel.k_elevator=0;
                        SRV_Channel.k_rudder=0;
                        AP_Motors_output();
                    end
                end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    {ENUM_FlightTaskMode.Fix2Rotor_Mode,ENUM_FlightTaskMode.StallRecovery}
        switch plane_mode
            case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1,ENUM_plane_mode.V10s}
                if(PathMode~=ENUM_FlightTaskMode.Fix2Rotor_Mode)
                    PathMode=ENUM_FlightTaskMode.Fix2Rotor_Mode;
                    relax_attitude_controllers();
                    SINS.curr_pos(1:2)=[0 0];
                     init_vel_controller_xyz_c2c();
                    AC_PosControl.roll_target=0;
                    AC_PosControl.pitch_target=pitch_target_p2c;
                    SRV_Channel.k_throttle=0;
                    Fix2Rotor_delay=0;
                    Fix2Rotor_delay_flag=0;
                end
                if(Fix2Rotor_delay_flag==0&&Copter_Plane.State==ENUM_State.E_Plane)
                    if(Fix2Rotor_delay>=0.250)
                        Fix2Rotor_delay_flag=1;
                        relax_attitude_controllers();
                        SINS.curr_pos(1:2)=[0 0];
                        init_vel_controller_xyz_c2c();
                        Copter_Plane.State=ENUM_State.E_Copter;
                    else
                        Fix2Rotor_delay=Fix2Rotor_delay+dt;
                        Fix2Rotor_delay_flag=0;
                    end
                    Plane.nav_pitch_cd  = AC_PosControl.pitch_target;
                    Plane.nav_roll_cd   = AC_PosControl.roll_target;
                    stabilize();
                    AP_Motors_output();
                    AP_Motors.throttle_filter =0.05;
                    SRV_Channel.pwm_out=[1050 1050 1050 1050];
                else
                    update_z_controller();
                    roll_target  = AC_PosControl.roll_target;
                    pitch_target = AC_PosControl.pitch_target;
                    target_yaw_rate = 0;
                    input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                    rate_controller_run();
                    if(aspeed>aspeed_c2p)
                        Plane.nav_pitch_cd  = AC_PosControl.pitch_target;
                        Plane.nav_roll_cd   = AC_PosControl.roll_target;
                        stabilize()
                        SRV_Channel.k_aileron=SRV_Channel.k_aileron*p_plane_c2p;
                        SRV_Channel.k_elevator=SRV_Channel.k_elevator*p_plane_c2p;
                        SRV_Channel.k_rudder=SRV_Channel.k_rudder*p_plane_c2p;
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                        AP_Motors_output();
                    else
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                        set_Plane_SRV_to_zero();
                        AP_Motors_output();
                    end
                end
            case {ENUM_plane_mode.V1000}
                if(PathMode~=ENUM_FlightTaskMode.Fix2Rotor_Mode)
                    PathMode=ENUM_FlightTaskMode.Fix2Rotor_Mode;
                    relax_attitude_controllers();
                    SINS.curr_pos(1:2)=[0 0];
                    init_vel_controller_xyz();
                    AC_PosControl.roll_target=0;
                    AC_PosControl.pitch_target=pitch_target_p2c;
                    SRV_Channel.k_throttle=0;
                    Fix2Rotor_delay=0;
                    Fix2Rotor_delay_flag=0;
                    AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c;
                end
                if(Copter_Plane.State==ENUM_State.E_Plane)
                    if(Fix2Rotor_delay_flag==0)
                        Plane.nav_pitch_cd  = AC_PosControl.pitch_target;
                        Plane.nav_roll_cd   = AC_PosControl.roll_target;
                        stabilize();
                        AP_Motors_output();
                        Fix2Rotor_delay=Fix2Rotor_delay+dt;
                        if(Fix2Rotor_delay>Fix2Rotor_delay_s)
                            Fix2Rotor_delay_flag=1;
                            Fix2Rotor_delay=0;
                        end
                    else
                        Fix2Rotor_delay_flag=1;
                        set_throttle_out( 0,false, 10);
                        AP_Motors_output();
                        SRV_Channel.tail_tilt=Copter_Plane.tail_tilt_p2c;
                        Fix2Rotor_delay=Fix2Rotor_delay+dt;
                        if(Fix2Rotor_delay>0.2)
                            Copter_Plane.State=ENUM_State.E_Copter;
                            Fix2Rotor_delay=0;
                            relax_attitude_controllers();
                            SINS.curr_pos(1:2)=[0 0];
                            init_vel_controller_xyz_c2c();
                        end
                    end
                else
                    tail_error=-SRV_Channel.tail_tilt;
                    tail_error=constrain_value(tail_error,-Copter_Plane.tail_tilt_rate*dt,Copter_Plane.tail_tilt_rate*dt);
                    SRV_Channel.tail_tilt=SRV_Channel.tail_tilt+tail_error;
                    if(aspeed>aspeed_c2p)
                        Plane.nav_pitch_cd  = AC_PosControl.pitch_target;
                        Plane.nav_roll_cd   = AC_PosControl.roll_target;
                        stabilize()
                        SRV_Channel.k_aileron=SRV_Channel.k_aileron*p_plane_c2p;
                        SRV_Channel.k_elevator=SRV_Channel.k_elevator*p_plane_c2p;
                        SRV_Channel.k_rudder=SRV_Channel.k_rudder*p_plane_c2p;
                        AP_Motors.yaw_in=constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                        AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c;
                        AP_Motors_output();
                    else
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                        AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=POSCONTROL_THROTTLE_CUTOFF_FREQ;
                        set_Plane_SRV_to_zero();
                        AP_Motors_output();
                    end
                    update_z_controller();
                    roll_target  = AC_PosControl.roll_target;
                    pitch_target = AC_PosControl.pitch_target;
                    target_yaw_rate = 0;
                    input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                    rate_controller_run();
                end
                
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    {ENUM_FlightTaskMode.PathFollowMode,ENUM_FlightTaskMode.InterestPointMode }
        if(PathMode~=ENUM_FlightTaskMode.PathFollowMode)
            PathMode=ENUM_FlightTaskMode.PathFollowMode;
            Copter_Plane.State=ENUM_State.E_Plane;
%             hgt_dem_cm=height*100;
%             Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init_p2p();
        end
        hgt_dem_cm = Copter_Plane.hgt_dem_cm;
        if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos
            climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
            climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        else
            climb_rate_cms=0;
            hgt_dem_cm=PathModeOut_sl.heightCmd;
        end
        Copter_Plane.climb_rate_cms  = climb_rate_cms;
        Copter_Plane.hgt_dem_cm = hgt_dem_cm;
        if( PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.CircleHoverMode)
            center_WP=PathModeOut_sl.turnCenterLL(1:2);
            update_loiter( center_WP,   radius,   loiter_direction)
        else
            prev_WP=PathModeOut_sl.prePathPoint_LLA(1:2);
            next_WP=PathModeOut_sl.curPathPoint_LLA(1:2);
            if(heading_hold)
                update_heading_hold(PathModeOut_sl.headingCmd*HD*100)
            else
                update_waypoint( prev_WP,  next_WP,  dist_min)
            end
        end
        plane_run();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.GoHomeMode
        if(PathMode~=ENUM_FlightTaskMode.GoHomeMode)
            PathMode=ENUM_FlightTaskMode.GoHomeMode;
            Copter_Plane.center_WP=curr_loc;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init_p2p();
            SINS.curr_pos(1:2)=[0 0];
            init_vel_controller_xyz_c2c();
            relax_attitude_controllers_c2c();
        end
        switch Copter_Plane.State
            case ENUM_State.E_Plane
                hgt_dem_cm = Copter_Plane.hgt_dem_cm;
                if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos
                    climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                    hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                    climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                    hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                else
                    climb_rate_cms=0;
                    hgt_dem_cm=PathModeOut_sl.heightCmd;
                end
                prev_WP=PathModeOut_sl.prePathPoint_LLA(1:2);
                next_WP=PathModeOut_sl.curPathPoint_LLA(1:2);
                if(Copter_Plane.heading_hold||(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.HeadingTrackMode))
                    update_heading_hold(PathModeOut_sl.headingCmd*HD*100);
                else
                    update_waypoint( prev_WP,  next_WP,  dist_min)
                end
                Copter_Plane.climb_rate_cms  = climb_rate_cms;
                Copter_Plane.hgt_dem_cm = hgt_dem_cm;
                plane_run();
            case ENUM_State.E_Copter
                AC_PosControl.pos_target(1:2)=[0 0];
                loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;
                if (PathModeOut_sl.heightCmd-AC_PosControl.pos_target(3))>error_pos
                    climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                    set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);
                elseif (PathModeOut_sl.heightCmd-AC_PosControl.pos_target(3))<-error_pos
                    climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                    set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);
                else
                    climb_rate_cms=0;
                    AC_PosControl.pos_target(3)=PathModeOut_sl.heightCmd;
                end
                Copter_Plane.climb_rate_cms  = climb_rate_cms;
                copter_run();
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    case    ENUM_FlightTaskMode.GroundStandByMode
        if(PathMode~=ENUM_FlightTaskMode.GroundStandByMode)
            PathMode=ENUM_FlightTaskMode.GroundStandByMode;
            Copter_Plane.State=ENUM_State.E_Copter;
            relax_attitude_controllers();
            Land_throttle_in=throttle_ground;
            set_throttle_out( throttle_ground,false, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
        end
        TakeOffMode_delay=0;
        TakeOffMode_delay_flag=0;
        set_Plane_SRV_to_zero();
        throttle_in_error=-Land_throttle_in;
        throttle_in_error=constrain_value(throttle_in_error,-throttle_off_rate*dt,throttle_off_rate*dt);
        Land_throttle_in=Land_throttle_in+throttle_in_error;
        Land_throttle_in=max(Land_throttle_in,0.1);
        set_throttle_out(Land_throttle_in, 0, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
        roll_target=0;
        pitch_target=0;
        target_yaw_rate=0;
        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
        rate_controller_run();
        AP_Motors_output();  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case    ENUM_FlightTaskMode.VerticalMove
        if(PathMode~=ENUM_FlightTaskMode.VerticalMove)
            PathMode=ENUM_FlightTaskMode.VerticalMove;
%             hgt_dem_cm=height*100;
%             Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init_p2p();
            SINS.curr_pos(1:2)=[0 0];
            init_vel_controller_xyz();
            relax_attitude_controllers();
            
        end
        switch Copter_Plane.State
            case ENUM_State.E_Plane
                hgt_dem_cm = Copter_Plane.hgt_dem_cm;
                if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos
                    climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                    hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                    climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                    hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                else
                    climb_rate_cms=0;
                    hgt_dem_cm=PathModeOut_sl.heightCmd;
                end
                center_WP=PathModeOut_sl.turnCenterLL(1:2);
                update_loiter( center_WP,   radius,   loiter_direction)
                Copter_Plane.climb_rate_cms  = climb_rate_cms;
                Copter_Plane.hgt_dem_cm = hgt_dem_cm;
                plane_run();
            case ENUM_State.E_Copter
                AC_PosControl.pos_target(1:2)=[0 0];
                loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;
                if (PathModeOut_sl.heightCmd-AC_PosControl.pos_target(3))>error_pos
                    climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                    set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);
                elseif (PathModeOut_sl.heightCmd-AC_PosControl.pos_target(3))<-error_pos
                    climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                    set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);
                else
                    climb_rate_cms=0;
                    AC_PosControl.pos_target(3)=PathModeOut_sl.heightCmd;
                end
                Copter_Plane.climb_rate_cms  = climb_rate_cms;
                copter_run();
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    otherwise
        Copter_Plane.State = ENUM_State.E_Copter;
        copter_run();
end

end

