function  run_remote()
global roll_target
global pitch_target
global target_yaw_rate
global throttle_in
global climb_rate_cms
global roll_target_pilot
global pitch_target_pilot
global nav_pitch_cd
global latAccDem
global throttle_dem
global tail_tilt
global mode
global algo_remote_ct_st
dead=0.05;
            mode=algo_remote_ct_st.mode;
            algo_remote_ct_st.roll = deadzonef(algo_remote_ct_st.roll,dead,1);
            algo_remote_ct_st.pitch = deadzonef(algo_remote_ct_st.pitch,dead,1);
            algo_remote_ct_st.yaw = deadzonef(algo_remote_ct_st.yaw,dead,1);           
    switch mode
        case 1
            roll_target=algo_remote_ct_st.roll*4500;
            pitch_target=algo_remote_ct_st.pitch*4500;
            target_yaw_rate=algo_remote_ct_st.yaw*20000;
            throttle_in=algo_remote_ct_st.throttle;              
        case 2
            roll_target=algo_remote_ct_st.roll*4500;
            pitch_target=algo_remote_ct_st.pitch*4500;
            target_yaw_rate=algo_remote_ct_st.yaw*20000;
            climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;            
        case 3
            roll_target_pilot=algo_remote_ct_st.roll*400;
            pitch_target_pilot=-algo_remote_ct_st.pitch*400;
            target_yaw_rate=algo_remote_ct_st.yaw*20000;          
            climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;            
        case 4
            latAccDem=algo_remote_ct_st.roll*9.8;
            nav_pitch_cd=algo_remote_ct_st.pitch*2000;
            throttle_dem=algo_remote_ct_st.throttle;
        case 5
            latAccDem=algo_remote_ct_st.roll*9.8;
            climb_rate_cms=algo_remote_ct_st.pitch*400;
        case 6
            climb_rate_cms=algo_remote_ct_st.pitch*400;
        case 7
            roll_target=algo_remote_ct_st.roll*4500;
            pitch_target=algo_remote_ct_st.pitch*4500;
            target_yaw_rate=algo_remote_ct_st.yaw*20000;
            climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;                      
            tail_tilt=algo_remote_ct_st.tilt_anglein;
        case 8
            climb_rate_cms=algo_remote_ct_st.pitch*400;
    end
    
end

