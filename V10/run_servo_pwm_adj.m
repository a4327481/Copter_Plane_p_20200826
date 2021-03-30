function run_servo_pwm_adj()
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
global algo_remote_ct_st
global Test_w
global SRV_Channel
global Copter_Plane

mode              = Copter_Plane.mode;

k_rudder        = SRV_Channel.k_rudder;
k_flap          = SRV_Channel.k_flap;

if(mode==4||mode==5||mode==6||mode==8)
k_rudder=constrain_value(algo_remote_ct_st.yaw*4500+k_rudder,-4500,4500);
end
k_flap=k_flap+Test_w.k_flap;

SRV_Channel.k_rudder           = k_rudder;
SRV_Channel.k_flap             = k_flap;
Copter_Plane.mode              = mode;



end

