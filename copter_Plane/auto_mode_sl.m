function auto_mode_sl()
%auto flight
%mode auto


global dt
global curr_alt
global curr_loc
global Plane 
global AC_Attitude
global AC_PosControl
global AP_Motors
global AP_L1
global AP_TECS
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
thr_out_min                           = AC_PosControl.thr_out_min;
attitude_target_quat                  = AC_Attitude.attitude_target_quat;
latAccDem                             = AP_L1.latAccDem;
throttle_dem                          = AP_TECS.throttle_dem;       
last_throttle_dem                     = AP_TECS.last_throttle_dem; 
throttle_cruise                       = AP_TECS.throttle_cruise;    
spdWeight                             = AP_TECS.spdWeight;      
integTHR_state                        = AP_TECS.integTHR_state;     

yaw_in                                = AP_Motors.yaw_in;    
throttle_filter                       = AP_Motors.throttle_filter;
throttle_in                           = AP_Motors.throttle_in;

tail_tilt                             = SRV_Channel.tail_tilt; 
k_aileron                             = SRV_Channel.k_aileron;
k_elevator                            = SRV_Channel.k_elevator;
k_rudder                              = SRV_Channel.k_rudder;
k_throttle                            = SRV_Channel.k_throttle;
pwm_out                               = SRV_Channel.pwm_out;
k_flap                                = SRV_Channel.k_flap;

climb_rate_cms                           = Copter_Plane.climb_rate_cms;
loc_origin                               = Copter_Plane.loc_origin;
EAS_dem_cm                               = Copter_Plane.EAS_dem_cm;
hgt_dem_cm                               = Copter_Plane.hgt_dem_cm;
p_tilt_pitch_target                      = Copter_Plane.p_tilt_pitch_target;
center_WP                                = Copter_Plane.center_WP;
radius                                   = Copter_Plane.radius;
loiter_direction                         = Copter_Plane.loiter_direction;
prev_WP                                  = Copter_Plane.prev_WP;
next_WP                                  = Copter_Plane.next_WP;
dist_min                                 = Copter_Plane.dist_min;
p_plane_c2p                              = Copter_Plane.p_plane_c2p;
yaw_max_c2p                              = Copter_Plane.yaw_max_c2p;
inint_hgt                                = Copter_Plane.inint_hgt;
tail_tilt_c2p                            = Copter_Plane.tail_tilt_c2p;
tail_tilt_p2c                            = Copter_Plane.tail_tilt_p2c;
tail_tilt_rate                           = Copter_Plane.tail_tilt_rate;
aspeed_c2p                               = Copter_Plane.aspeed_c2p;
aspeed_c2ps                              = Copter_Plane.aspeed_c2ps;
p_plane_p2c                              = Copter_Plane.p_plane_p2c;
pitch_target_p2c                         = Copter_Plane.pitch_target_p2c;
pitch_target_c2p                         = Copter_Plane.pitch_target_c2p;
k_throttle_c2p                           = Copter_Plane.k_throttle_c2p;
Fix2Rotor_delay_s                        = Copter_Plane.Fix2Rotor_delay_s;
p_k_elevator_c2p                         = Copter_Plane.p_k_elevator_c2p;
POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c      = Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c;
POSCONTROL_THROTTLE_CUTOFF_FREQ          = Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ;
throttle_ground                          = Copter_Plane.throttle_ground;
throttle_off_rate                        = Copter_Plane.throttle_off_rate;
take_off_land                            = Copter_Plane.take_off_land;
vel_forward_integrator                   = Copter_Plane.vel_forward_integrator;

heading_hold                             = Copter_Plane.heading_hold;
k_flap_TakeOff                           = Copter_Plane.k_flap_TakeOff;
k_flap_Land                              = Copter_Plane.k_flap_Land;
disable_AP_roll_integrator            = Copter_Plane.disable_AP_roll_integrator;          
disable_AP_pitch_integrator           = Copter_Plane.disable_AP_pitch_integrator;
disable_AP_yaw_integrator             = Copter_Plane.disable_AP_yaw_integrator;
disable_AP_rate_roll_gains_D          = Copter_Plane.disable_AP_rate_roll_gains_D;
disable_AP_rate_pitch_roll_ff         = Copter_Plane.disable_AP_rate_pitch_roll_ff;
disable_AP_rate_pitch_gains_D         = Copter_Plane.disable_AP_rate_pitch_gains_D;
disable_AP_rate_yaw_K_FF              = Copter_Plane.disable_AP_rate_yaw_K_FF;
POSCONTROL_ACC_Z_FILT_HZ_c2p          = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p;
POSCONTROL_ACC_Z_FILT_HZ              = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ;
height                                =curr_alt/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global HD
global curr_pos
global aspeed
global rot_body_to_ned
global PathModeOut_sl
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mode auto
persistent WP_i
persistent PathMode
persistent uavMode %0:comper 1:plane
persistent Rotor2Fix_delay
persistent Rotor2Fix_delay_flag
persistent Fix2Rotor_delay
persistent Fix2Rotor_delay_flag
persistent TakeOffMode_delay
persistent TakeOffMode_delay_flag
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       error_pos=8;
    if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.TakeOffMode||PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.LandMode)
      take_off_land=1;
    else
      take_off_land=0;
    end
    
    if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.Rotor2Fix_Mode)
      thr_out_min  = Copter_Plane.thr_out_min_c2p ;
    else
      thr_out_min =  Copter_Plane.thr_out_min;
    end

   if(PathModeOut_sl.flightTaskMode~=ENUM_FlightTaskMode.Fix2Rotor_Mode)
      AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=POSCONTROL_THROTTLE_CUTOFF_FREQ;
    end
      
    if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.TakeOffMode)
        k_flap=k_flap_TakeOff;
    elseif(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.LandMode)
        k_flap=k_flap_Land;
    else
        k_flap=0;
    end
      
    if( uavMode==1)%% mode :mode=1 ,Plane;mode=0 , Copter. disable plane I,vel_forward_integrator=0;
        disable_AP_roll_integrator=false;
        disable_AP_pitch_integrator=false;
        disable_AP_yaw_integrator=false;
        disable_AP_rate_pitch_roll_ff =false;
        disable_AP_rate_pitch_gains_D =false;
        disable_AP_rate_yaw_K_FF =false;
        disable_AP_rate_roll_gains_D =false;            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        vel_forward_integrator=0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        AC_PosControl.pid_accel_z.flags_reset_filter=true;
        AC_PosControl.pid_vel_xy.flags_reset_filter=true;
        rate_pitch_pid.flags_reset_filter=true;
        rate_roll_pid.flags_reset_filter=true;
        rate_yaw_pid.flags_reset_filter=true;
    else       
        disable_AP_roll_integrator=true;
        disable_AP_pitch_integrator=true;
        disable_AP_yaw_integrator=true;
        disable_AP_rate_pitch_roll_ff =true;
        disable_AP_rate_pitch_gains_D =true;
        disable_AP_rate_yaw_K_FF =true;
        disable_AP_rate_roll_gains_D =true;        
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        switch PathModeOut_sl.flightTaskMode
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.TakeOffMode
                if((PathMode==ENUM_FlightTaskMode.GroundStandByMode)&&(TakeOffMode_delay_flag==0))
                       TakeOffMode_delay=TakeOffMode_delay+dt;
                       tail_tilt=0;
                       k_aileron=0;
                       k_elevator=0;
                       k_rudder=0; 
                       throttle_in=0.1;
                       throttle_filter=0.1;
                       pwm_out=[1100 1100 1100 1100];  
                       AC_PosControl.pid_accel_z.flags_reset_filter=true;
                       AC_PosControl.pid_vel_xy.flags_reset_filter=true;
                       rate_pitch_pid.flags_reset_filter=true;
                       rate_roll_pid.flags_reset_filter=true;
                       rate_yaw_pid.flags_reset_filter=true;
                       if(TakeOffMode_delay>1)
                           TakeOffMode_delay_flag=1;
                       end
                else
                         if(PathMode~=ENUM_FlightTaskMode.TakeOffMode)
                            PathMode=ENUM_FlightTaskMode.TakeOffMode;
                            uavMode=0;
                            pos_target(3)=max(curr_alt,10);
                            vel_desired(3)=0;
                            loc_origin=curr_loc;
                            curr_pos(1:2)=[0 0];
                            pos_target(1:2)=[0 0];
                            attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                            target_yaw_rate=0;
                         else
                            curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100; 
                         end
                         if(curr_alt<100)
                             AC_PosControl.pid_accel_z.disable_integrator=true;
                             AC_PosControl.pid_vel_xy.disable_integrator=true;
                             rate_pitch_pid.disable_integrator=true;
                             rate_roll_pid.disable_integrator=true;
                             rate_yaw_pid.disable_integrator=true;
                         end
                         climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                         pos_target(3)=PathModeOut_sl.heightCmd;                         
                        if(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.RotorGoUpDownBySpeed)
                            take_off_land=1;
                             set_alt_target_from_climb_rate_ffg1(climb_rate_cms, dt, 0,1);        
                             copter_run();
                        elseif(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.SpotHoverMode)
                            take_off_land=0;
                             set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);   
                             copter_run();
                        else
                            take_off_land=0;
                            set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);   
                            copter_run();
                        end
                end
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.LandMode
                if(PathMode~=ENUM_FlightTaskMode.LandMode)
                    PathMode=ENUM_FlightTaskMode.LandMode;
                    uavMode=0;
                    loc_origin=curr_loc;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                else
                    pos_target(1:2)=[0 0]; 
                    loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100; 
                end
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg1(climb_rate_cms, dt, 0,-1); 
                 if(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.RotorGoUpDownWithHorizonPosFree)
                      copter_run_posfree();
                 else
                      copter_run();
                 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%               
            case    ENUM_FlightTaskMode.HoverAdjustMode
                  if(PathMode~=ENUM_FlightTaskMode.HoverAdjustMode)
                    PathMode=ENUM_FlightTaskMode.HoverAdjustMode;
                    uavMode=0;
                    loc_origin=curr_loc;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                  else
                    pos_target(1:2)=[0 0];  
                    loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100; 
                  end               
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);                                      
                 copter_run();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.HoverUpMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
%                       center_WP=curr_loc;                     
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
                 center_WP=PathModeOut_sl.turnCenterLL(1:2);
                 update_loiter( center_WP,   radius,   loiter_direction)
                 plane_run();  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.HoverDownMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
%                      center_WP=curr_loc;                     
                end 
                center_WP=PathModeOut_sl.turnCenterLL(1:2);
                update_loiter( center_WP,   radius,   loiter_direction) 
                if(PathModeOut_sl.rollCmd==0)
                    latAccDem=0;
                end
                if(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.HoverSliderMode)
                    integTHR_state=0;
                    spdWeight=2; 
                    plane_run();
                    inint_hgt=1;
                    hgt_dem_cm=height*100; 
                    
                else                    
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
                    plane_run();
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.AirStandByMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
%                      center_WP=curr_loc;                     
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
                center_WP=PathModeOut_sl.turnCenterLL(1:2); 
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
                            nav_pitch_cd=pitch_target_c2p;
                            k_throttle=throttle_cruise * 0.01+k_throttle_c2p;
                            throttle_dem=throttle_cruise * 0.01+k_throttle_c2p;
                            last_throttle_dem=throttle_cruise * 0.01+k_throttle_c2p;
                            stabilize()
                            output_to_motors_plane();
                            k_elevator=k_elevator+p_k_elevator_c2p;
                      else
                             inint_hgt=1;
                             hgt_dem_cm=height*100;
                             %%%%
                             pitch_target=0;
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
                         pitch_target_temp=p_tilt_pitch_target*tail_tilt_temp;
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
                         pitch_target_temp=p_tilt_pitch_target*tail_tilt_temp;
                         update_z_controller();
                         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                         rate_controller_run();
                         %%%%
                         AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
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
                    uavMode=1;
                    pos_target(3) = curr_alt;
                    k_throttle=0;
                    vel_desired(3)=0;
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225
                    target_yaw_rate=0;  
                    roll_target=0;
                    pitch_target=pitch_target_p2c;
                    Fix2Rotor_delay_flag=0;
                    Fix2Rotor_delay=0;
                    AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c;
                 end
                 if(uavMode==1)
                     if(Fix2Rotor_delay_flag==0)
                         nav_pitch_cd=pitch_target;
                         nav_roll_cd=roll_target;
                         stabilize()
                         output_to_motors_plane();
                         Fix2Rotor_delay=Fix2Rotor_delay+dt;
                         if(Fix2Rotor_delay>Fix2Rotor_delay_s)
                             Fix2Rotor_delay_flag=1;
                             Fix2Rotor_delay=0;
                         end
                     else                                                
                         Fix2Rotor_delay_flag=1;
                         nav_pitch_cd=pitch_target;
                         nav_roll_cd=roll_target;
                         stabilize()                        
                         throttle_filter=0;
                         throttle_in=0;
                         set_throttle_out(throttle_in, 1, 10);
                         AP_MotorsMulticopter_output();
                         tail_tilt=tail_tilt_p2c;
                         Fix2Rotor_delay=Fix2Rotor_delay+dt;
                          if(Fix2Rotor_delay>0.2)
                             uavMode=0;
                             Fix2Rotor_delay=0;
                             pos_target(3) = curr_alt;
                             attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225

                         end                              
                     end     
                 else
                     input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                     rate_controller_run();
                     tail_error=-tail_tilt;
                     tail_error=constrain_value(tail_error,-tail_tilt_rate*dt,tail_tilt_rate*dt);
                     tail_tilt=tail_tilt+tail_error;
                     if(aspeed>aspeed_c2p)
                         nav_pitch_cd=pitch_target;
                         nav_roll_cd=roll_target;
                         stabilize()
                         k_aileron=k_aileron*p_plane_p2c;
                         k_elevator=k_elevator*p_plane_p2c;
                         k_rudder=k_rudder*p_plane_p2c;
                         yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                         AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                         AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c;
                         update_z_controller();
%                          throttle_in=0.3;
%                          throttle_filter=0.3;
%                          set_throttle_out(throttle_in, 0, 10);
                         AP_MotorsMulticopter_output();
                     else
                         AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                         AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ=POSCONTROL_THROTTLE_CUTOFF_FREQ;
                         update_z_controller();
%                          throttle_in=0.3;
%                          throttle_filter=0.3;  
%                          set_throttle_out(throttle_in, 0, 10);
                         k_aileron=0;
                         k_elevator=0;
                         k_rudder=0;
                         AP_MotorsMulticopter_output();
                     end
                 end             
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            case    ENUM_FlightTaskMode.PathFollowMode               
                if(PathMode~=ENUM_FlightTaskMode.PathFollowMode)
                     PathMode=ENUM_FlightTaskMode.PathFollowMode;
                     uavMode=1;
                     inint_hgt=1;
                     center_WP=curr_loc;                     
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
                     prev_WP=PathModeOut_sl.prePathPoint_LLA(1:2);
                     next_WP=PathModeOut_sl.curPathPoint_LLA(1:2);
                    if(heading_hold) 
                        update_heading_hold(PathModeOut_sl.headingCmd*HD*100)
                    else                       
                        update_waypoint( prev_WP,  next_WP,  dist_min)
                    end
                    plane_run();
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
            case    ENUM_FlightTaskMode.GoHomeMode 
                 if(PathMode~=ENUM_FlightTaskMode.GoHomeMode)
                     PathMode=ENUM_FlightTaskMode.GoHomeMode;
                     inint_hgt=1;
                     center_WP=curr_loc;
                     loc_origin=curr_loc;
                     pos_target(3) = curr_alt;
                     vel_desired(3)=0;
                     curr_pos(1:2)=[0 0];
                     pos_target(1:2)=[0 0];
                     attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                     target_yaw_rate=0;        
                 end 
                 switch uavMode
                 case 1
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
                         prev_WP=center_WP;
                         next_WP=PathModeOut_sl.curPathPoint_LLA(1:2);                                        
                         if(heading_hold||(PathModeOut_sl.flightControlMode==ENUM_FlightControlMode.HeadingTrackMode))                        
                             update_heading_hold(PathModeOut_sl.headingCmd*HD*100);                  
                         else                          
                             update_waypoint( prev_WP,  next_WP,  dist_min);                
                         end                         
                        plane_run();
                  case 0
                    pos_target(1:2)=[0 0]; 
                    loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;                 
                     if (PathModeOut_sl.heightCmd-pos_target(3))>error_pos         
                         climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                         set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);
                     elseif (PathModeOut_sl.heightCmd-pos_target(3))<-error_pos
                         climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                         set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);                 
                     else
                         climb_rate_cms=0;
                         pos_target(3)=PathModeOut_sl.heightCmd;
                     end                                        
                        copter_run();   
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                
                 end
            case    ENUM_FlightTaskMode.GroundStandByMode 
                if(PathMode~=ENUM_FlightTaskMode.GroundStandByMode)
                     PathMode=ENUM_FlightTaskMode.GroundStandByMode;
                     throttle_in=throttle_ground;
                end 
                     TakeOffMode_delay=0;
                     TakeOffMode_delay_flag=0;
                     tail_tilt=0;
                     k_aileron=0;
                     k_elevator=0;
                     k_rudder=0;
                     roll_target=0;
                     pitch_target=0;
                     target_yaw_rate=0;                   
                     throttle_in_error=-throttle_in;
                     throttle_in_error=constrain_value(throttle_in_error,-throttle_off_rate*dt,throttle_off_rate*dt);
                     throttle_in=throttle_in+throttle_in_error; 
                     throttle_in=max(throttle_in,0.1);
                     set_throttle_out(throttle_in, 0, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
                     input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                     rate_controller_run();
                     AP_MotorsMulticopter_output();
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
             case    ENUM_FlightTaskMode.StallRecovery 
                 if(PathMode~=ENUM_FlightTaskMode.StallRecovery)
                    PathMode=ENUM_FlightTaskMode.StallRecovery;
                    uavMode=0;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225
                    target_yaw_rate=0;  
                    tail_tilt=tail_tilt_p2c;
                    roll_target=0;
                    pitch_target=pitch_target_p2c;
                end
                 update_z_controller();
                 tail_tilt_temp=constrain_value(tail_tilt,-3000,700);
                 pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
                 input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                 rate_controller_run();
                 tail_error=-tail_tilt;
                 if(abs(tail_error)>100)
                     throttle_filter=0;
                     throttle_in=0;
                 end
                 tail_error=constrain_value(tail_error,-tail_tilt_rate*dt,tail_tilt_rate*dt);
                 tail_tilt=tail_tilt+tail_error;

                    nav_pitch_cd=pitch_target;
                    nav_roll_cd=roll_target;
                    stabilize()
                    k_aileron=k_aileron*p_plane_c2p;
                    k_elevator=k_elevator*p_plane_c2p;
                    k_rudder=k_rudder*p_plane_c2p;
                    yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                     AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ;
                    AP_MotorsMulticopter_output();
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
             case    ENUM_FlightTaskMode.VerticalMove                             
                 if(PathMode~=ENUM_FlightTaskMode.VerticalMove)
                     PathMode=ENUM_FlightTaskMode.VerticalMove;
                     inint_hgt=1;
                     center_WP=curr_loc;
                     loc_origin=curr_loc;
                     pos_target(3) = curr_alt;
                     vel_desired(3)=0;
                     curr_pos(1:2)=[0 0];
                     pos_target(1:2)=[0 0];
                     attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                     target_yaw_rate=0;        
                 end 
                 switch uavMode
                 case 1
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
                        plane_run();  
                  case 0
                    pos_target(1:2)=[0 0]; 
                    loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( curr_loc,loc_origin)*100;                 
                     if (PathModeOut_sl.heightCmd-pos_target(3))>error_pos         
                         climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                         set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);
                     elseif (PathModeOut_sl.heightCmd-pos_target(3))<-error_pos
                         climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                         set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0);                 
                     else
                         climb_rate_cms=0;
                         pos_target(3)=PathModeOut_sl.heightCmd;
                     end                                        
                        copter_run();   
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                
                 end
            otherwise         
                 copter_run();
        end
        
        	
Plane.aerodynamic_load_factor                      = aerodynamic_load_factor;
Plane.nav_pitch_cd                                 = nav_pitch_cd;
Plane.nav_roll_cd                                  = nav_roll_cd;

AC_PosControl.roll_target                          = roll_target;
AC_PosControl.pitch_target                         = pitch_target;
AC_PosControl.target_yaw_rate                      = target_yaw_rate;
AC_PosControl.pos_target                           = pos_target;
AC_PosControl.vel_desired                          = vel_desired;
AC_PosControl.thr_out_min                          = thr_out_min;

AC_Attitude.attitude_target_quat                   = attitude_target_quat;

AP_L1.latAccDem                                    = latAccDem;
AP_TECS.throttle_dem                               = throttle_dem;       
AP_TECS.last_throttle_dem                          = last_throttle_dem; 
AP_TECS.throttle_cruise                            = throttle_cruise;    
AP_TECS.spdWeight                                  = spdWeight;      
AP_TECS.integTHR_state                             = integTHR_state;     

AP_Motors.yaw_in                                   = yaw_in;    
AP_Motors.throttle_filter                          = throttle_filter;
AP_Motors.throttle_in                              = throttle_in;
											    
SRV_Channel.tail_tilt                              = tail_tilt; 
SRV_Channel.k_aileron                              = k_aileron;
SRV_Channel.k_elevator                             = k_elevator;
SRV_Channel.k_rudder                               = k_rudder;
SRV_Channel.k_throttle                             = k_throttle;
SRV_Channel.pwm_out                                = pwm_out;
SRV_Channel.k_flap                                 = k_flap;		
											    
Copter_Plane.climb_rate_cms                           = climb_rate_cms;
Copter_Plane.loc_origin                               = loc_origin;
Copter_Plane.EAS_dem_cm                               = EAS_dem_cm;
Copter_Plane.hgt_dem_cm                               = hgt_dem_cm;
Copter_Plane.p_tilt_pitch_target                      = p_tilt_pitch_target;
Copter_Plane.center_WP                                = center_WP;
Copter_Plane.radius                                   = radius;
Copter_Plane.loiter_direction                         = loiter_direction;
Copter_Plane.prev_WP                                  = prev_WP;
Copter_Plane.next_WP                                  = next_WP;
Copter_Plane.dist_min                                 = dist_min;
Copter_Plane.p_plane_c2p                              = p_plane_c2p;
Copter_Plane.yaw_max_c2p                              = yaw_max_c2p;
Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p             = POSCONTROL_ACC_Z_FILT_HZ_c2p;
Copter_Plane.inint_hgt                                = inint_hgt;
Copter_Plane.tail_tilt_c2p                            = tail_tilt_c2p;
Copter_Plane.tail_tilt_p2c                            = tail_tilt_p2c;
Copter_Plane.tail_tilt_rate                           = tail_tilt_rate;
Copter_Plane.aspeed_c2p                               = aspeed_c2p;
Copter_Plane.aspeed_c2ps                              = aspeed_c2ps;
Copter_Plane.p_plane_p2c                              = p_plane_p2c;
Copter_Plane.pitch_target_p2c                         = pitch_target_p2c;
Copter_Plane.pitch_target_c2p                         = pitch_target_c2p;
Copter_Plane.k_throttle_c2p                           = k_throttle_c2p;
Copter_Plane.Fix2Rotor_delay_s                        = Fix2Rotor_delay_s;
Copter_Plane.p_k_elevator_c2p                         = p_k_elevator_c2p;
Copter_Plane.POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c      = POSCONTROL_THROTTLE_CUTOFF_FREQ_p2c;
Copter_Plane.throttle_ground                          = throttle_ground;
Copter_Plane.throttle_off_rate                        = throttle_off_rate;
Copter_Plane.take_off_land                            = take_off_land;
Copter_Plane.vel_forward_integrator                   = vel_forward_integrator;

Copter_Plane.heading_hold                             = heading_hold;
Copter_Plane.k_flap_TakeOff                           = k_flap_TakeOff;
Copter_Plane.k_flap_Land                              = k_flap_Land;
Copter_Plane.disable_AP_roll_integrator            = disable_AP_roll_integrator;          
Copter_Plane.disable_AP_pitch_integrator           = disable_AP_pitch_integrator;
Copter_Plane.disable_AP_yaw_integrator             = disable_AP_yaw_integrator;
Copter_Plane.disable_AP_rate_roll_gains_D          = disable_AP_rate_roll_gains_D;
Copter_Plane.disable_AP_rate_pitch_roll_ff         = disable_AP_rate_pitch_roll_ff;
Copter_Plane.disable_AP_rate_pitch_gains_D         = disable_AP_rate_pitch_gains_D;
Copter_Plane.disable_AP_rate_yaw_K_FF              = disable_AP_rate_yaw_K_FF;
end

