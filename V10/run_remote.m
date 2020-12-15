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
global pwm_tail
dead=0.05;
            if(algo_remote_ct_st.isRemoteConnected||algo_remote_ct_st.mode==9||algo_remote_ct_st.mode==10)
                mode=algo_remote_ct_st.mode;
            else
                if(algo_remote_ct_st.mode==1||algo_remote_ct_st.mode==2||algo_remote_ct_st.mode==3||algo_remote_ct_st.mode==7)
                    mode=3;
                elseif((algo_remote_ct_st.mode==0||algo_remote_ct_st.mode==4||algo_remote_ct_st.mode==5||algo_remote_ct_st.mode==6||algo_remote_ct_st.mode==8))
                    mode=8;
                end               
            end
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
            roll_target_pilot=algo_remote_ct_st.roll*5000;
            pitch_target_pilot=-algo_remote_ct_st.pitch*5000;
            target_yaw_rate=algo_remote_ct_st.yaw*20000;          
            climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;            
        case 4
            latAccDem=algo_remote_ct_st.roll*9.8;
            nav_pitch_cd=algo_remote_ct_st.pitch*2000;
            throttle_dem=algo_remote_ct_st.throttle;
        case 5
            latAccDem=algo_remote_ct_st.roll*9.8;
            climb_rate_cms=algo_remote_ct_st.pitch*600;
        case 6
            climb_rate_cms=algo_remote_ct_st.pitch*600;
        case 7
            roll_target=algo_remote_ct_st.roll*4500;
            pitch_target=algo_remote_ct_st.pitch*4500;
            target_yaw_rate=algo_remote_ct_st.yaw*20000;
            climb_rate_cms = deadzonef(algo_remote_ct_st.throttle-0.5,dead,0.5)*800;                      
            tail_tilt=algo_remote_ct_st.tail_anglein;
            pwm_tail=algo_remote_ct_st.tail_throttle_pwm;
        case 8
            climb_rate_cms=algo_remote_ct_st.pitch*600;
    end
    
end

