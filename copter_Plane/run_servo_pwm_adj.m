function run_servo_pwm_adj()
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
global algo_remote_ct_st
global Test_w
global SRV_Channel
global Copter_Plane

Mode            = Copter_Plane.Mode;
k_rudder        = SRV_Channel.k_rudder;

if(Mode==4||Mode==5||Mode==6||Mode==8)
    k_rudder=constrain_value(algo_remote_ct_st.yaw*4500+k_rudder,-4500,4500);
end

if(Mode ==7)
    SRV_Channel.tail_tilt=algo_remote_ct_st.tail_anglein;
    SRV_Channel.pwm_tail=algo_remote_ct_st.tail_throttle_pwm;
end

SRV_Channel.k_flap             = SRV_Channel.k_flap +Test_w.k_flap;
SRV_Channel.k_rudder           = k_rudder;



end

