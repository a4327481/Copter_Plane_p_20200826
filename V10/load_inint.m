function load_inint()
%load innint 
 
global ATC_RAT_PIT_I_inint
global ATC_RAT_RLL_I_inint
global ATC_RAT_YAW_I_inint
global POSCONTROL_ACC_Z_I_inint
global POSCONTROL_ACC_Z_FILT_HZ_inint
global POSCONTROL_THROTTLE_CUTOFF_FREQ_inint
global gains_D_pitch_inint
global roll_ff_pitch_inint
global gains_D_roll_inint
global K_FF_yaw_inint
global spdWeight_inint
global POSCONTROL_VEL_XY_I_inint
global roll_limit_cd_inint
global thr_out_min_inint   

global ATC_RAT_PIT_I
global ATC_RAT_RLL_I
global ATC_RAT_YAW_I
global POSCONTROL_ACC_Z_I
global POSCONTROL_ACC_Z_FILT_HZ
global POSCONTROL_THROTTLE_CUTOFF_FREQ
global gains_D_pitch
global roll_ff_pitch
global  gains_D_roll
global  K_FF_yaw
global  spdWeight
global POSCONTROL_VEL_XY_I
global roll_limit_cd
global thr_out_min   

    ATC_RAT_PIT_I                   =ATC_RAT_PIT_I_inint;
    ATC_RAT_RLL_I                   =ATC_RAT_RLL_I_inint;
    ATC_RAT_YAW_I                   =ATC_RAT_YAW_I_inint;
    POSCONTROL_VEL_XY_I             =POSCONTROL_VEL_XY_I_inint;
    POSCONTROL_ACC_Z_I              =POSCONTROL_ACC_Z_I_inint;
    
    roll_ff_pitch                   =roll_ff_pitch_inint;
    K_FF_yaw                        =K_FF_yaw_inint;
    gains_D_pitch                   =gains_D_pitch_inint;
    gains_D_roll                    =gains_D_roll_inint;
    
    POSCONTROL_ACC_Z_FILT_HZ        =POSCONTROL_ACC_Z_FILT_HZ_inint;
    POSCONTROL_THROTTLE_CUTOFF_FREQ =POSCONTROL_THROTTLE_CUTOFF_FREQ_inint;
    thr_out_min                     =thr_out_min_inint;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    spdWeight                       =spdWeight_inint;
    roll_limit_cd                   =roll_limit_cd_inint;

 end

