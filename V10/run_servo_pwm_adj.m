function run_servo_pwm_adj()
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
global algo_remote_ct_st
global k_rudder
global k_aileron
global k_elevator
global k_flap
global k_aileronL
global k_aileronR
global mode
global Test_w
if(mode==4||mode==5||mode==6||mode==8)
k_rudder=constrain_value(algo_remote_ct_st.yaw*4500+k_rudder,-4500,4500);
end
k_aileronL=-k_aileron;
k_aileronR=k_aileron;
k_rudder=k_rudder+Test_w.k_rudder;
k_aileron=k_aileron+Test_w.k_aileron;
k_elevator=k_elevator+Test_w.k_elevator;
k_flap=k_flap+Test_w.k_flap;

end

