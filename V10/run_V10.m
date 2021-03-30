function run_V10()

global dt
global Plane 
global AC_Attitude
global AC_PosControl
global AP_Motors
global AP_rate_pitch
global AP_rate_yaw
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
attitude_target_quat                  = AC_Attitude.attitude_target_quat;
hgt_dem                               = AP_TECS.hgt_dem;

yaw_in                                = AP_Motors.yaw_in;    
throttle_in                           = AP_Motors.throttle_in;

tail_tilt                             = SRV_Channel.tail_tilt; 
k_aileron                             = SRV_Channel.k_aileron;
k_elevator                            = SRV_Channel.k_elevator;
k_rudder                              = SRV_Channel.k_rudder;


climb_rate_cms                           = Copter_Plane.climb_rate_cms;
loc_origin                               = Copter_Plane.loc_origin;
current_loc                              = Copter_Plane.current_loc;
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
yaw_max_c2p                              = Copter_Plane.yaw_max_c2p;
POSCONTROL_ACC_Z_FILT_HZ_c2p             = Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p;
inint_hgt                                = Copter_Plane.inint_hgt;
aspeed_c2p                               = Copter_Plane.aspeed_c2p;
mode                                     = Copter_Plane.mode;
inint                                    = Copter_Plane.inint;
roll_target_pilot                        = Copter_Plane.roll_target_pilot;
pitch_target_pilot                       = Copter_Plane.pitch_target_pilot;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global arspeed_filt
global arspeed_temp
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global curr_pos
global aspeed
global yaw
global rot_body_to_ned
global height
global curr_alt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global disable_integrator_pitch
global disable_integrator_roll
global disable_integrator_yaw
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% quad _4a1 
persistent mode_state
persistent WP_i
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if isempty(mode_state)
        mode_state = 0;
    end
    
    if isempty(WP_i)
        WP_i = 1;
    end  
           

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
  if(mode==1||mode==2||mode==3||mode==7)%%disable plane I
    disable_integrator_pitch=1;
    disable_integrator_roll=1;
    disable_integrator_yaw=1;
    AP_rate_pitch.roll_ff=0;
    AP_rate_yaw.K_FF=0;
  else
    disable_integrator_pitch=0;
    disable_integrator_roll=0;
    disable_integrator_yaw=0;
    AP_rate_pitch.roll_ff=roll_ff_pitch_inint;
    AP_rate_yaw.K_FF=K_FF_yaw_inint;
  end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
      
    if(mode_state==3||mode_state==9||mode_state==10)
        AC_PosControl.pid_accel_z.ki=POSCONTROL_ACC_Z_I_inint;
        AC_PosControl.pid_vel_xy.ki=POSCONTROL_VEL_XY_I_inint;
        rate_pitch_pid.ki=ATC_RAT_PIT_I_inint;
        rate_roll_pid.ki=ATC_RAT_RLL_I_inint;
        rate_yaw_pid.ki=ATC_RAT_YAW_I_inint;    
    else
        AC_PosControl.pid_accel_z.ki=0;
        AC_PosControl.pid_vel_xy.ki=0;
        rate_pitch_pid.ki=0;
        rate_roll_pid.ki=0;
        rate_yaw_pid.ki=0; 
        pid_accel_z_reset_filter=1;
        pid_vel_xy_reset_filter=1;
        rate_pitch_pid_reset_filter=1;
        rate_roll_pid_reset_filter=1;
        rate_yaw_pid_reset_filter=1;
    end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    if(inint)
    setup_motors_4a1() ;
    inint=0;
    end
 
  %% mode 1 :copter Stabilize,2:copter althold ,3:copter poshold,4:Plane Stabilize 5:Plane TECS; 6：Plane L1  
  updata_cl();
  rc=1/(2*pi*arspeed_filt);
  alpha=dt/(dt+rc);
  arspeed_temp=arspeed_temp+(aspeed-arspeed_temp)*alpha;
  aspeed=arspeed_temp;
  
  tail_tilt=0;
switch mode
    case 1 %copter Stabilize
        if(mode_state~=1)
            attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225  
            mode_state=1;
             k_aileron=0;
             k_elevator=0;
             k_rudder=0;
             k_throttle=0;
        end                
        set_throttle_out(throttle_in, 1, AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ);
        input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
        rate_controller_run();
        AP_MotorsMulticopter_output_4a1();
    case 2 %copter althold
        if(mode_state~=2)
            pos_target(3) = curr_alt;
            vel_desired(3)=0;
            mode_state=2; 
             k_aileron=0;
             k_elevator=0;
             k_rudder=0;
             k_throttle=0;
        end
         set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0)
         update_z_controller();
         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
         rate_controller_run();
         AP_MotorsMulticopter_output_4a1();
    case 3 %copter xyz
        if(mode_state~=3)
            loc_origin=current_loc;
            curr_pos(1:2)=[0 0];
            pos_target(1:2)=[0 0];
            pos_target(3) = curr_alt;
            vel_desired(3)=0;
            mode_state=3;
             k_aileron=0;
             k_elevator=0;
             k_rudder=0;
             k_throttle=0;
        else
%             pos_target(1:2)=[0 0];
            curr_pos(1:2)=get_vector_xy_from_origin_NE( current_loc,loc_origin)*100; 
        end
         vel_desired(1)=pitch_target_pilot/10*cos(yaw)-roll_target_pilot/10*sin(yaw);
         vel_desired(2)=pitch_target_pilot/10*sin(yaw)+roll_target_pilot/10*cos(yaw);
         
         if( (abs(pitch_target_pilot)>0) || (abs(roll_target_pilot)>0)||(abs(target_yaw_rate)>0))
             temp_yaw_rate=0;
         else
             temp_yaw_rate=get_weathervane_yaw_rate_cds();
         end                 
         set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0)
         update_vel_controller_xy();
         update_z_controller();
         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate+temp_yaw_rate);
         rate_controller_run();
         AP_MotorsMulticopter_output_4a1();
    case 4 %Plane Stabilize
        if(mode_state~=4)         
            mode_state=4;
        end
%         calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        output_to_motors_plane_4a1();
    case 5 %Plane TECS
        if(mode_state~=5)
            hgt_dem_cm=height*100;
            hgt_dem=height;
            inint_hgt=1;
            mode_state=5;
        else
             hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        end    
        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        %update_loiter( center_WP,   radius,   loiter_direction)
        %update_waypoint( prev_WP,  next_WP,  dist_min)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        output_to_motors_plane_4a1();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 6 %Plane L1 waypoint
                    if(mode_state~=6)
                         hgt_dem_cm=height*100;
                         hgt_dem=height;
                         inint_hgt=1;
                         mode_state=6; 
                    else
                         hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                    end    
            %         prev_WP=loc(WP_i,2:3);
            %         next_WP=loc(WP_i+1,2:3); 
                     prev_WP=[loc.lat(WP_i) loc.lon(WP_i)];
                     next_WP=[loc.lat(WP_i+1) loc.lon(WP_i+1)];        
                    AB = get_distance_NE(next_WP,current_loc);
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
            %         update_loiter( center_WP,   radius,   loiter_direction)
                     update_waypoint( prev_WP,  next_WP,  dist_min)
                    calc_nav_pitch();
                    calc_nav_roll()
                    calc_throttle()
                    stabilize()
                    output_to_motors_plane_4a1();
    case 7 %copter plane Manual
        if(mode_state~=7)
            pos_target(3) = curr_alt;
            vel_desired(3)=0;
            mode_state=7;
            k_throttle=0;
         attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225
        end
         set_alt_target_from_climb_rate_ff(climb_rate_cms, dt, 0)
         update_z_controller();
%          tail_tilt_temp=constrain_value((pwm_tail-pwm_min)/(pwm_max-pwm_min)*-9000,-3000,0);
%          pitch_target_temp=pitch_target+p_tilt_pitch_target*tail_tilt_temp;
         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
         rate_controller_run();
         if(aspeed>aspeed_c2p)
            nav_pitch_cd=pitch_target;
            nav_roll_cd=roll_target;
            stabilize()
            k_aileron=k_aileron*p_plane_c2p;
            k_elevator=k_elevator*p_plane_c2p;
            k_rudder=k_rudder*p_plane_c2p;
            yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
            AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_c2p;
         else          
             AC_PosControl.pid_accel_z.filt_E_hz=POSCONTROL_ACC_Z_FILT_HZ_inint;
             k_aileron=0;
             k_elevator=0;
             k_rudder=0;
         end
         calc_throttle();
        AP_MotorsMulticopter_output_4a1();

      case 8 %Plane L1 loiter
        if(mode_state~=8)
             hgt_dem_cm=height*100;
             hgt_dem=height;
             inint_hgt=1;
             center_WP=current_loc;
             mode_state=8; 
        else
             hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
        end    
        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        update_loiter( center_WP,   radius,   loiter_direction)
%          update_waypoint( prev_WP,  next_WP,  dist_min)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        output_to_motors_plane_4a1(); 

        case 9 %Auto test  
        if(mode_state~=9)
             mode_state=9; 
        end
        auto_mode_4a1();
        case 10 %Auto sl  
        if(mode_state~=10)
             mode_state=10; 
        end
         auto_mode_sl_4a1();  
    otherwise
        mode_state=mode;
        %copter Stabilize
%         set_throttle_out(throttle_in, 1, POSCONTROL_THROTTLE_CUTOFF_FREQ);
%         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
%         rate_controller_run();
%         AP_MotorsMulticopter_output();
end


Plane.aerodynamic_load_factor                     = aerodynamic_load_factor;
Plane.nav_pitch_cd                                = nav_pitch_cd;
Plane.nav_roll_cd                                 = nav_roll_cd;

AC_PosControl.roll_target                         = roll_target;
AC_PosControl.pitch_target                        = pitch_target;
AC_PosControl.target_yaw_rate                     = target_yaw_rate;
AC_PosControl.pos_target                          = pos_target;
AC_PosControl.vel_desired                         = vel_desired;
AC_Attitude.attitude_target_quat                  = attitude_target_quat;
AP_TECS.hgt_dem                                   = hgt_dem;

AP_Motors.yaw_in                                  = yaw_in;    
AP_Motors.throttle_in                             = throttle_in;

SRV_Channel.tail_tilt                             = tail_tilt; 
SRV_Channel.k_aileron                             = k_aileron;
SRV_Channel.k_elevator                            = k_elevator;
SRV_Channel.k_rudder                              = k_rudder;
Copter_Plane.climb_rate_cms                       = climb_rate_cms;
Copter_Plane.loc_origin                           = loc_origin;
Copter_Plane.current_loc                          = current_loc;
Copter_Plane.EAS_dem_cm                           = EAS_dem_cm;
Copter_Plane.hgt_dem_cm                           = hgt_dem_cm;
Copter_Plane.center_WP                            = center_WP;
Copter_Plane.radius                               = radius;
Copter_Plane.loiter_direction                     = loiter_direction;
Copter_Plane.prev_WP                              = prev_WP;
Copter_Plane.next_WP                              = next_WP;
Copter_Plane.dist_min                             = dist_min;
Copter_Plane.loc                                  = loc;
Copter_Plane.L1_radius                            = L1_radius;
Copter_Plane.p_plane_c2p                          = p_plane_c2p;
Copter_Plane.yaw_max_c2p                          = yaw_max_c2p;
Copter_Plane.POSCONTROL_ACC_Z_FILT_HZ_c2p         = POSCONTROL_ACC_Z_FILT_HZ_c2p;
Copter_Plane.inint_hgt                            = inint_hgt;
Copter_Plane.aspeed_c2p                           = aspeed_c2p;
Copter_Plane.mode                                 = mode;
Copter_Plane.inint                                = inint;
Copter_Plane.roll_target_pilot                    = roll_target_pilot ;
Copter_Plane.pitch_target_pilot                   = pitch_target_pilot;
end

