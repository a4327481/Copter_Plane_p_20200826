%     pitch=(0:90)/57.3;
%     roll=0;
%     cos_tilt = cos(pitch) * cos(roll);
%     len=length(pitch);
%     for i=1:len
%     inverted_factor(i) = constrain_value(2.0 * cos_tilt(i), 0.0, 1.0);
%     boost_factor(i) = 1.0 ./ constrain_value(cos_tilt(i), 0.5, 1.0); 
%     end
%     throttle_out = throttle_in * inverted_factor * boost_factor;
global POSCONTROL_POS_Z_P
global POSCONTROL_ACCEL_Z
global POSCONTROL_SPEED_DOWN
global POSCONTROL_SPEED_UP
global dt
pos_error_test=(0:500)';
len=length(pos_error_test);
out=zeros(len,1);

for i=1:len
out(i)=sqrt_controller(pos_error_test(i), POSCONTROL_POS_Z_P, POSCONTROL_ACCEL_Z, dt);
end
hold on
plot(pos_error_test,out)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
        leash_up_z = calc_leash_length(POSCONTROL_SPEED_UP, POSCONTROL_ACCEL_Z, POSCONTROL_POS_Z_P);
        leash_down_z = calc_leash_length(-POSCONTROL_SPEED_DOWN, POSCONTROL_ACCEL_Z, POSCONTROL_POS_Z_P);