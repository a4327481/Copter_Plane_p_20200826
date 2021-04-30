function Copter_Plane_run()

global dt
global Plane
global AC_PosControl
global AP_Motors
global AC_rate_pitch_pid
global AC_rate_roll_pid
global AC_rate_yaw_pid
global SRV_Channel
global Copter_Plane
global SINS
global plane_mode

aerodynamic_load_factor                  = Plane.aerodynamic_load_factor;
climb_rate_cms                           = Copter_Plane.climb_rate_cms;
loc_origin                               = Copter_Plane.loc_origin;
EAS_dem_cm                               = Copter_Plane.EAS_dem_cm;
hgt_dem_cm                               = Copter_Plane.hgt_dem_cm;
center_WP                                = Copter_Plane.center_WP;
radius                                   = Copter_Plane.radius;
loiter_direction                         = Copter_Plane.loiter_direction;
prev_WP                                  = Copter_Plane.prev_WP;
next_WP                                  = Copter_Plane.next_WP;
dist_min                                 = Copter_Plane.dist_min;
loc                                      = Copter_Plane.loc;
L1_radius                                = Copter_Plane.L1_radius;
p_plane_c2p                              = Copter_Plane.p_plane_c2p;
p_tilt_pitch_target                      = Copter_Plane.p_tilt_pitch_target;
yaw_max_c2p                              = Copter_Plane.yaw_max_c2p;
POSCONTROL_ACC_Z_FILT_HZ_c2p             = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p;
POSCONTROL_ACC_Z_FILT_HZ                 = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ;
aspeed_c2p                               = Copter_Plane.aspeed_c2p;
roll_target_pilot                        = Copter_Plane.roll_target_pilot;
pitch_target_pilot                       = Copter_Plane.pitch_target_pilot;
tail_tilt                                = SRV_Channel.tail_tilt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
height                                = SINS.curr_alt/100;
aspeed                                = SINS.aspeed;
yaw                                   = SINS.yaw;
curr_loc                              = SINS.curr_loc;
%%%%%%%%%%%%%%%%% quad _4a1
persistent WP_i
persistent Mode_State
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isempty(WP_i)
    WP_i = 1;
end
if isempty(Mode_State)
    Mode_State = ENUM_Mode.MANUAL;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(Copter_Plane.State == ENUM_State.Copter)%%disable plane I
    Copter_Plane.disable_AP_roll_integrator                 = true;
    Copter_Plane.disable_AP_pitch_integrator                = true;
    Copter_Plane.disable_AP_yaw_integrator                  = true;
    Copter_Plane.disable_AP_rate_pitch_gains_D              = true;
    Copter_Plane.disable_AP_rate_roll_gains_D               = true;
    Copter_Plane.disable_AP_rate_yaw_K_FF                   = true;
    Copter_Plane.disable_AP_rate_pitch_roll_ff              = true;
else
    Copter_Plane.disable_AP_roll_integrator                 = false;
    Copter_Plane.disable_AP_pitch_integrator                = false;
    Copter_Plane.disable_AP_yaw_integrator                  = false;
    Copter_Plane.disable_AP_rate_pitch_gains_D              = false;
    Copter_Plane.disable_AP_rate_roll_gains_D               = false;
    Copter_Plane.disable_AP_rate_yaw_K_FF                   = false;
    Copter_Plane.disable_AP_rate_pitch_roll_ff              = false;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(Copter_Plane.Mode==ENUM_Mode.Copter_POS_HOLD||Copter_Plane.Mode==ENUM_Mode.AUTO_TEST||Copter_Plane.Mode==ENUM_Mode.AUTO)
else
    AC_PosControl.pid_accel_z.disable_integrator  = false;
    AC_PosControl.pid_vel_xy.disable_integrator   = false;
    AC_rate_pitch_pid.disable_integrator          = false;
    AC_rate_roll_pid.disable_integrator           = false;
    AC_rate_yaw_pid.disable_integrator            = false; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% mode 1 :copter Stabilize,2:copter althold ,3:copter poshold,4:Plane Stabilize 5:Plane TECS; 6：Plane L1

switch Copter_Plane.Mode
    case ENUM_Mode.Copter_STABILIZE %copter Stabilize
        if(Mode_State~=ENUM_Mode.Copter_STABILIZE)
            Mode_State=ENUM_Mode.Copter_STABILIZE;
            Copter_Plane.State = ENUM_State.Copter;
            relax_attitude_controllers();
            set_Plane_SRV_to_zero();
        end
        set_throttle_outg( true, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
        roll_target                           = AC_PosControl.roll_target;
        pitch_target                          = AC_PosControl.pitch_target;
        target_yaw_rate                       = AC_PosControl.target_yaw_rate;
        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
        rate_controller_run();
        AP_Motors_output();
    case ENUM_Mode.Copter_ALT_HOLD %copter althold
        if(Mode_State~=ENUM_Mode.Copter_ALT_HOLD)
            Mode_State=ENUM_Mode.Copter_ALT_HOLD;
            Copter_Plane.State = ENUM_State.Copter;
            relax_attitude_controllers();
            init_vel_controller_xyz();
            set_Plane_SRV_to_zero();
        end
        set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0)
        update_z_controller();
        roll_target                           = AC_PosControl.roll_target;
        pitch_target                          = AC_PosControl.pitch_target;
        target_yaw_rate                       = AC_PosControl.target_yaw_rate;
        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
        rate_controller_run();
        AP_Motors_output();
    case ENUM_Mode.Copter_POS_HOLD %copter xyz
        if(Mode_State~=ENUM_Mode.Copter_POS_HOLD)
            Mode_State=ENUM_Mode.Copter_POS_HOLD;
            Copter_Plane.State = ENUM_State.Copter;
            Copter_Plane.loc_origin=curr_loc;
            SINS.curr_pos(1:2)=[0 0];
            relax_attitude_controllers();
            init_vel_controller_xyz();
            set_Plane_SRV_to_zero();
        else
            SINS.curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;
        end
        AC_PosControl.vel_desired(1)=pitch_target_pilot/10*cos(yaw)-roll_target_pilot/10*sin(yaw);
        AC_PosControl.vel_desired(2)=pitch_target_pilot/10*sin(yaw)+roll_target_pilot/10*cos(yaw); 
        set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0)
        update_vel_controller_xy();
        update_z_controller();
        roll_target                           = AC_PosControl.roll_target;
        pitch_target                          = AC_PosControl.pitch_target;
        target_yaw_rate                       = AC_PosControl.target_yaw_rate;
        if( (abs(pitch_target_pilot)>0) || (abs(roll_target_pilot)>0)||(abs(target_yaw_rate)>0))
            temp_yaw_rate=0;
        else
            temp_yaw_rate=get_weathervane_yaw_rate_cds();
        end
        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate+temp_yaw_rate);
        rate_controller_run();
        AP_Motors_output();
    case ENUM_Mode.Plane_STABILIZE %Plane Stabilize
        if(Mode_State~=ENUM_Mode.Plane_STABILIZE)
            Mode_State=ENUM_Mode.Plane_STABILIZE;
            Copter_Plane.State = ENUM_State.Plane;
        end
        calc_nav_roll()
        calc_throttle()
        stabilize()
        AP_Motors_output();
    case ENUM_Mode.Plane_TECS %Plane TECS
        if(Mode_State~=ENUM_Mode.Plane_TECS)
            Mode_State=ENUM_Mode.Plane_TECS;
            Copter_Plane.State = ENUM_State.Plane;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init();
        else
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        end
        update_50hz();      
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        AP_Motors_output();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case ENUM_Mode.Plane_L1_WAYPOINT %Plane L1 waypoint
        if(Mode_State~=ENUM_Mode.Plane_L1_WAYPOINT)
            Mode_State=ENUM_Mode.Plane_L1_WAYPOINT;
            Copter_Plane.State = ENUM_State.Plane;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init();
        else
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        end
        prev_WP=[loc.lat(WP_i) loc.lon(WP_i)];
        next_WP=[loc.lat(WP_i+1) loc.lon(WP_i+1)];
        AB = get_distance_NE(next_WP,curr_loc);
        AB_length = norm(AB,2);
        if((AB_length<L1_radius))
            if((loc.lat(WP_i+2,1)~=0&&loc.lon(WP_i+2,1)~=0))
                WP_i=WP_i+1;
            else
                WP_i=2;
            end
        end
        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        update_waypoint( prev_WP,  next_WP,  dist_min)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        AP_Motors_output();
    case ENUM_Mode.Copter_Plane_MANUAL %copter plane Manual
        if(Mode_State~=ENUM_Mode.Copter_Plane_MANUAL)
            Mode_State=ENUM_Mode.Copter_Plane_MANUAL;
            Copter_Plane.State = ENUM_State.Copter;
            SINS.curr_pos(1:2)=[0 0];
            relax_attitude_controllers();
            init_vel_controller_xyz();
            SRV_Channel.k_throttle     = 0;
        end
        set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0)
        update_z_controller();
        roll_target                           = AC_PosControl.roll_target;
        pitch_target                          = AC_PosControl.pitch_target;
        target_yaw_rate                       = AC_PosControl.target_yaw_rate;
        switch plane_mode
            case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1,ENUM_plane_mode.V10s}
                input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);                
            case {ENUM_plane_mode.V1000}
                tail_tilt_temp=constrain_value(tail_tilt,-3000,700);
                pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
                input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
            otherwise
                input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
        end
        rate_controller_run();
        if(aspeed>aspeed_c2p)
            Plane.nav_pitch_cd = pitch_target;
            Plane.nav_roll_cd = roll_target;
            stabilize()
            SRV_Channel.k_aileron   = SRV_Channel.k_aileron*p_plane_c2p;
            SRV_Channel.k_elevator  = SRV_Channel.k_elevator*p_plane_c2p;
            SRV_Channel.k_rudder    = SRV_Channel.k_rudder*p_plane_c2p;
            AP_Motors.yaw_in        = constrain_value(AP_Motors.yaw_in,-yaw_max_c2p,yaw_max_c2p);
            AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
        else
            AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
            SRV_Channel.k_aileron   = 0;
            SRV_Channel.k_elevator  = 0;
            SRV_Channel.k_rudder    = 0;
        end
        calc_throttle();
        AP_Motors_output();      
    case ENUM_Mode.Plane_L1_LOITER %Plane L1 loiter
        if(Mode_State~=ENUM_Mode.Plane_L1_LOITER)
            Mode_State=ENUM_Mode.Plane_L1_LOITER;
            Copter_Plane.State = ENUM_State.Plane;
            hgt_dem_cm=height*100;
            Copter_Plane.hgt_dem_cm = hgt_dem_cm;
            AP_TECS_init();
            center_WP=curr_loc;
        else
            hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        end
        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        update_loiter( center_WP,   radius,   loiter_direction)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        AP_Motors_output();      
    case ENUM_Mode.AUTO_TEST %Auto test
        if(Mode_State~=ENUM_Mode.AUTO_TEST)
            Mode_State=ENUM_Mode.AUTO_TEST;
        end
        auto_mode();
    case ENUM_Mode.AUTO %Auto sl
        if(Mode_State~=ENUM_Mode.AUTO)
            Mode_State=ENUM_Mode.AUTO;
        end
        auto_mode();
    otherwise
        Mode_State=Copter_Plane.Mode;
end
Copter_Plane.hgt_dem_cm                           = hgt_dem_cm;
Copter_Plane.center_WP                            = center_WP;
Copter_Plane.prev_WP                              = prev_WP;
Copter_Plane.next_WP                              = next_WP;
end

