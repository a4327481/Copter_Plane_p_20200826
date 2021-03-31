function auto_mode()
%auto flight
%mode auto

global dt
global Plane 
global AC_Attitude
global AC_PosControl
global AP_Motors
global AP_L1
global AP_rate_pitch
global AP_rate_yaw
global rate_pitch_pid
global rate_roll_pid
global rate_yaw_pid
global SRV_Channel
global Copter_Plane

aerodynamic_load_factor               = Plane.aerodynamic_load_factor;
nav_pitch_cd                          = Plane.nav_pitch_cd;
nav_roll_cd                           = Plane.nav_roll_cd;

roll_target                           = AC_PosControl.roll_target;
pitch_target                          = AC_PosControl.pitch_target;
target_yaw_rate                       = AC_PosControl.target_yaw_rate;
pos_target                            = AC_PosControl.pos_target;
vel_desired                           = AC_PosControl.vel_desired;
attitude_target_quat                  = AC_Attitude.attitude_target_quat;

latAccDem                             = AP_L1.latAccDem;

yaw_in                                = AP_Motors.yaw_in;    
throttle_filter                       = AP_Motors.throttle_filter;
throttle_in                           = AP_Motors.throttle_in;

tail_tilt                             = SRV_Channel.tail_tilt; 
k_aileron                             = SRV_Channel.k_aileron;
k_elevator                            = SRV_Channel.k_elevator;
k_rudder                              = SRV_Channel.k_rudder;

climb_rate_cms       = Copter_Plane.climb_rate_cms;
loc_origin           = Copter_Plane.loc_origin;
current_loc          = Copter_Plane.current_loc;
EAS_dem_cm           = Copter_Plane.EAS_dem_cm;
hgt_dem_cm           = Copter_Plane.hgt_dem_cm;
p_tilt_pitch_target  = Copter_Plane.p_tilt_pitch_target;
center_WP            = Copter_Plane.center_WP;

prev_WP                            = Copter_Plane.prev_WP;
next_WP                            = Copter_Plane.next_WP;
dist_min                           = Copter_Plane.dist_min;
loc                                = Copter_Plane.loc;
L1_radius                          = Copter_Plane.L1_radius;
radius                             = Copter_Plane.radius;                               
loiter_direction                   = Copter_Plane.loiter_direction;
p_plane_c2p                        = Copter_Plane.p_plane_c2p;
yaw_max_c2p                        = Copter_Plane.yaw_max_c2p;
POSCONTROL_ACC_Z_FILT_HZ_c2p       = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p;
inint_hgt                          = Copter_Plane.inint_hgt;
tail_tilt_c2p                      = Copter_Plane.tail_tilt_c2p;
tail_tilt_p2c                      = Copter_Plane.tail_tilt_p2c;
tail_tilt_rate                     = Copter_Plane.tail_tilt_rate;
aspeed_c2p                         = Copter_Plane.aspeed_c2p;
aspeed_c2ps                        = Copter_Plane.aspeed_c2ps;
disable_integrator_pitch           = Copter_Plane.disable_integrator_pitch;          
disable_integrator_roll            = Copter_Plane.disable_integrator_roll;
disable_integrator_yaw              = Copter_Plane.disable_integrator_yaw;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global curr_pos
global aspeed
global yaw
global rot_body_to_ned
global height
global curr_alt
global PathModeOut_sl

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global roll_ff_pitch_inint
global K_FF_yaw_inint
global POSCONTROL_ACC_Z_FILT_HZ_inint
global POSCONTROL_ACC_Z_I_inint
global pid_accel_z_reset_filter
global POSCONTROL_VEL_XY_I_inint
global pid_vel_xy_reset_filter
global ATC_RAT_PIT_I_inint
global rate_pitch_pid_reset_filter
global ATC_RAT_RLL_I_inint
global rate_roll_pid_reset_filter
global ATC_RAT_YAW_I_inint
global rate_yaw_pid_reset_filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
persistent WP_i
persistent PathMode
persistent uavMode %0:comper 1:plane
persistent Rotor2Fix_delay
persistent Rotor2Fix_delay_flag


    if isempty(uavMode)
        uavMode = 0;
    end  
    
    if isempty(WP_i)
        WP_i = 1;
    end      
    if isempty(PathMode)
        PathMode = ENUM_FlightTaskMode.NoneFlightTaskMode;
    end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
  if isempty(Rotor2Fix_delay)
        Rotor2Fix_delay = 0;  
  end
  if isempty(Rotor2Fix_delay_flag)
        Rotor2Fix_delay_flag = 0;  
  end   
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
       error_pos=8;   
    if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.HoverAdjustMode||PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.TakeOffMode||PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.LandMode)

    else
        AC_PosControl.pid_accel_z.flags_reset_filter=true;
        AC_PosControl.pid_vel_xy.flags_reset_filter=true;
        rate_pitch_pid.flags_reset_filter=true;
        rate_roll_pid.flags_reset_filter=true;
        rate_yaw_pid.flags_reset_filter=true;
    end
    
    if( uavMode==1)%% mode= comper disable plane I
        disable_integrator_pitch=0;
        disable_integrator_roll=0;
        disable_integrator_yaw=0;       
        AP_rate_pitch.roll_ff=roll_ff_pitch_inint;
        AP_rate_yaw.K_FF=K_FF_yaw_inint; 
    else
        disable_integrator_pitch=1;
        disable_integrator_roll=1;
        disable_integrator_yaw=1;
        AP_rate_pitch.roll_ff=0;
        AP_rate_yaw.K_FF=0;  
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        switch PathModeOut_sl.flightTaskMode
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.TakeOffMode
                 if(PathMode~=ENUM_FlightTaskMode.TakeOffMode)
                    PathMode=ENUM_FlightTaskMode.TakeOffMode;
                    uavMode=0;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    loc_origin=current_loc;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                 else
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( current_loc,loc_origin)*100; 
                 end
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);        
                 copter_run();
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.LandMode
                if(PathMode~=ENUM_FlightTaskMode.LandMode)
                    PathMode=ENUM_FlightTaskMode.LandMode;
                    uavMode=0;
                    loc_origin=current_loc;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                else
%                     pos_target(1:2)=[0 0]; 
%                     loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( current_loc,loc_origin)*100; 
                end
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);              
                 copter_run();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            case    ENUM_FlightTaskMode.HoverAdjustMode
                  if(PathMode~=ENUM_FlightTaskMode.HoverAdjustMode)
                    PathMode=ENUM_FlightTaskMode.HoverAdjustMode;
                    uavMode=0;
                    loc_origin=current_loc;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                  else
%                     pos_target(1:2)=[0 0];  
%                     loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( current_loc,loc_origin)*100; 
                  end               
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);
                 vel_desired(1)=PathModeOut_sl.groundspeedCmd*cos(yaw);
                 vel_desired(2)=PathModeOut_sl.groundspeedCmd*sin(yaw);
                 copter_run();                                                       
            	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.HoverUpMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    center_WP=current_loc;                     
                end 
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
%                  center_WP=PathModeOut_sl.turnCenterLL(1:2);
                 update_loiter( center_WP,   radius,   loiter_direction)
                 plane_run();  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.HoverDownMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    center_WP=current_loc;                     
                end 
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
%                center_WP=PathModeOut_sl.turnCenterLL(1:2);
                update_loiter( center_WP,   radius,   loiter_direction)  
                if(PathModeOut_sl.rollCmd==0)
                    latAccDem=0;
                end
                plane_run();                 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.AirStandByMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    center_WP=current_loc;                     
                end 
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
%                center_WP=PathModeOut_sl.turnCenterLL(1:2); 
                update_loiter( center_WP,   radius,   loiter_direction) 
                plane_run();  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.Rotor2Fix_Mode 
                if(PathMode~=ENUM_FlightTaskMode.Rotor2Fix_Mode)
                    PathMode=ENUM_FlightTaskMode.Rotor2Fix_Mode;
                    uavMode=0;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225
                    target_yaw_rate=0; 
                    roll_target=0;
                    Rotor2Fix_delay_flag=0;
                    Rotor2Fix_delay=0;
                end
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 tail_error=tail_tilt_c2p-tail_tilt;
                 tail_error=constrain_value(tail_error,-tail_tilt_rate*dt,tail_tilt_rate*dt);
                 tail_tilt=tail_tilt+tail_error; 
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 if(uavMode==1)
                        if(Rotor2Fix_delay_flag)
                            Rotor2Fix_delay_flag=1;
                            nav_roll_cd=0;
                            update_50hz();
                            update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
                            calc_nav_pitch();                
                            calc_throttle()
                            stabilize()
                            output_to_motors_plane();    
                        else
                            inint_hgt=1;
                            hgt_dem_cm=height*100;        
                             %%%%
                             pitch_target=0;
                             tail_tilt_temp=tail_tilt_c2p;
                             pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
                             pitch_target_temp=0;
                             input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                             rate_controller_run();
                             %%%%  
                            nav_pitch_cd=pitch_target_temp;
                            nav_roll_cd=roll_target;
                            stabilize()
                            throttle_in=0;
                            yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                            AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                            AP_MotorsMulticopter_output();
                            if(throttle_filter<0.1)
                                tail_tilt=-9556;
                                Rotor2Fix_delay=Rotor2Fix_delay+dt;
                                if(Rotor2Fix_delay>0.2)
                                    Rotor2Fix_delay_flag=1;
                                end                              
                            end                        
                        end
                 else
                     if(aspeed>aspeed_c2ps)
                         %%%%
                         pitch_target=0;
                         tail_tilt_temp=tail_tilt_c2p;
                         pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
                         pitch_target_temp=0;
                         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                         rate_controller_run();
                         %%%%  
                        nav_pitch_cd=pitch_target_temp;
                        nav_roll_cd=roll_target;
                        stabilize()
                        throttle_in=0;
                        yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        AP_MotorsMulticopter_output();   
                        uavMode=1;                                                                 
                     elseif(aspeed>aspeed_c2p)
                         %%%%
                         tail_tilt_temp=constrain_value(tail_tilt,-3000,700);
                         if(pitch_target>0)
                             pitch_target=0;
                         end
                         pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
                         update_z_controller();
                         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                         rate_controller_run();
                         %%%%  
                        nav_pitch_cd=pitch_target_temp;
                        nav_roll_cd=roll_target;
                        stabilize()
                        k_aileron=k_aileron*p_plane_c2p;
                        k_elevator=k_elevator*p_plane_c2p;
                        k_rudder=k_rudder*p_plane_c2p;
                        yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        AP_MotorsMulticopter_output();
                     else
                         %%%%
                         tail_tilt_temp=constrain_value(tail_tilt,-3000,700);
                         if(pitch_target>0)
                             pitch_target=0;
                         end
                         pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
                         update_z_controller();
                         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                         rate_controller_run();
                         %%%%
                         AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_inint;
                         k_aileron=0;
                         k_elevator=0;
                         k_rudder=0;
                         AP_MotorsMulticopter_output();
                     end     
                 end
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.Fix2Rotor_Mode
                 if(PathMode~=ENUM_FlightTaskMode.Fix2Rotor_Mode)
                    PathMode=ENUM_FlightTaskMode.Fix2Rotor_Mode;
                    uavMode=0;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225
                    target_yaw_rate=0;  
                    tail_tilt=tail_tilt_p2c;
                    roll_target=0;
                    pitch_target=0;
                end
                 update_z_controller();
                 tail_tilt_temp=constrain_value(tail_tilt,-3000,700);
                 pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
                 input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                 rate_controller_run(); 
                 tail_error=-tail_tilt;
                 if(abs(tail_error)>100)
                     throttle_filter=0;
                     throttle_in=0;
                 end
                 tail_error=constrain_value(tail_error,-tail_tilt_rate*dt,tail_tilt_rate*dt);
                 tail_tilt=tail_tilt+tail_error;

                 if(aspeed>aspeed_c2p)
                    nav_pitch_cd=pitch_target;
                    nav_roll_cd=roll_target;
                    stabilize()
                    k_aileron=k_aileron*p_plane_c2p;
                    k_elevator=k_elevator*p_plane_c2p;
                    k_rudder=k_rudder*p_plane_c2p;
                    yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                    AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                    AP_MotorsMulticopter_output();
                 else          
                     AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_inint;
                     k_aileron=0;
                     k_elevator=0;
                     k_rudder=0;
                     AP_MotorsMulticopter_output();
                 end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
            case    ENUM_FlightTaskMode.PathFollowMode               
                if(PathMode~=ENUM_FlightTaskMode.PathFollowMode)
                     PathMode=ENUM_FlightTaskMode.PathFollowMode;
                     uavMode=1;
                     inint_hgt=1;
                     center_WP=current_loc;                     
                end 
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
                     prev_WP=[loc.lat(WP_i) loc.lon(WP_i)];
                     next_WP=[loc.lat(WP_i+1) loc.lon(WP_i+1)];        
                    AB = get_distance_NE(next_WP,current_loc);
                    AB_length = norm(AB,2);
                    if((AB_length<L1_radius))
                        if((loc.num(WP_i+2,1)~=99))       
                            WP_i=WP_i+1;      
                        else               
                            WP_i=2;
                        end
                    end
                    update_waypoint( prev_WP,  next_WP,  dist_min)
                    plane_run();   
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
            case    ENUM_FlightTaskMode.GoHomeMode 
                 if(PathMode~=ENUM_FlightTaskMode.GoHomeMode)
                     PathMode=ENUM_FlightTaskMode.GoHomeMode;
                     inint_hgt=1;
                     center_WP=current_loc;                     
                end 
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
                     prev_WP=[loc.lat(WP_i) loc.lon(WP_i)];
                     next_WP=[loc.lat(1) loc.lon(1)];        
                    update_waypoint( prev_WP,  next_WP,  dist_min)
                    plane_run();                   
            otherwise         
                 copter_run();
        end 
       
	   
Plane.aerodynamic_load_factor                   = aerodynamic_load_factor;
Plane.nav_pitch_cd                              = nav_pitch_cd;
Plane.nav_roll_cd                               = nav_roll_cd;

AC_PosControl.roll_target                       = roll_target;
AC_PosControl.pitch_target                      = pitch_target;
AC_PosControl.target_yaw_rate                   = target_yaw_rate;
AC_PosControl.pos_target                        = pos_target;
AC_PosControl.vel_desired                       = vel_desired;
AC_Attitude.attitude_target_quat                = attitude_target_quat;

AP_L1.latAccDem                                 = latAccDem;

AP_Motors.yaw_in                                = yaw_in;    
AP_Motors.throttle_filter                       = throttle_filter;
AP_Motors.throttle_in                           = throttle_in;

SRV_Channel.tail_tilt                           = tail_tilt; 
SRV_Channel.k_aileron                           = k_aileron;
SRV_Channel.k_elevator                          = k_elevator;
SRV_Channel.k_rudder                            = k_rudder;		
		
Copter_Plane.climb_rate_cms                     = climb_rate_cms;
Copter_Plane.loc_origin                         = loc_origin;
Copter_Plane.current_loc                        = current_loc;
Copter_Plane.EAS_dem_cm                         = EAS_dem_cm;
Copter_Plane.hgt_dem_cm                         = hgt_dem_cm;
Copter_Plane.p_tilt_pitch_target                = p_tilt_pitch_target;
Copter_Plane.center_WP                          = center_WP;
					               
Copter_Plane.prev_WP                            = prev_WP;
Copter_Plane.next_WP                            = next_WP;
Copter_Plane.dist_min                           = dist_min;
Copter_Plane.loc                                = loc;
Copter_Plane.L1_radius                          = L1_radius;
Copter_Plane.p_plane_c2p                        = p_plane_c2p;
Copter_Plane.yaw_max_c2p                        = yaw_max_c2p;
Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p       = POSCONTROL_ACC_Z_FILT_HZ_c2p;
Copter_Plane.inint_hgt                          = inint_hgt;
Copter_Plane.tail_tilt_c2p                      = tail_tilt_c2p;
Copter_Plane.tail_tilt_p2c                      = tail_tilt_p2c;
Copter_Plane.tail_tilt_rate                     = tail_tilt_rate;
Copter_Plane.aspeed_c2p                         = aspeed_c2p;
Copter_Plane.aspeed_c2ps                        = aspeed_c2ps; 
Copter_Plane.disable_integrator_pitch           = disable_integrator_pitch;          
Copter_Plane.disable_integrator_roll            = disable_integrator_roll;
Copter_Plane.disable_integrator_yaw             = disable_integrator_yaw;
end

