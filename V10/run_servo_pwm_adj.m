function run_servo_pwm_adj()
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
global algo_remote_ct_st
global k_rudder
global k_aileron
global k_aileronL
global k_aileronR
k_rudder=constrain_value(algo_remote_ct_st.yaw*4500+k_rudder,-4500,4500);
k_aileronL=-k_aileron;
k_aileronR=k_aileron;
end

