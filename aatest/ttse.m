% batt_voltage_filt=0:0.01:1;
%  lift_max = batt_voltage_filt * (1 - thrust_curve_expo) + thrust_curve_expo * batt_voltage_filt.* batt_voltage_filt;
%  hold on
%  plot(batt_voltage_filt,lift_max)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lift_max=1;
% for thrust_curve_expo=0.1:0.1:0.8
% thrust=0:0.01:1;
% throttle_ratio = ((thrust_curve_expo - 1.0) + sqrt((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo) + 4.0 * thrust_curve_expo * lift_max * thrust)) / (2.0 * thrust_curve_expo);
% hold on
% plot(thrust,throttle_ratio)
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% alpha=0:0.2:1;
% alpha=alpha';
% x=0:0.01:1;
% out= (1.0 - alpha) .* x + alpha.* (x.* x .* x);
% plot(x,out')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% thr_mid=0.5:0.1:0.8;
% thr_mid=thr_mid';
% alpha=0:0.01:1;
% alpha2 = alpha.*(thr_mid.*0+1) + 1.25 * (1.0 - alpha) .* (0.5 - thr_mid) / 0.5;
% plot(alpha,alpha2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p=1;
second_ord_lim=0;
error=0.903;
dt=0.005;
hold on
for error=-10:0.05:10 
intput=sqrt_controller( error,  p,  second_ord_lim, dt);
output = inv_sqrt_controller(  intput,   p,   second_ord_lim);
plot(error,error-output,'o')
end
